# local_align —— 多 session 增量建图

关键帧级的位姿图优化：同 session 内建帧间/回环约束，跨 session 用 scan-to-submap 把新
session 逐帧配到已有地图上，最后所有 session 的位姿一起联合优化。

**没有 submap 这一层**（区别于 glim 主程序）。每个关键帧一个位姿变量，submap 只是配准时
临时拼的 target，不是优化变量。

```bash
mkdir -p build && cd build && cmake .. && make -j8
cd .. && ./build/local_align --params params/main.yaml
```

本项目**已脱离 glim**：不需要 libglim.so，不需要先编译 glim，不需要 glim 的 `config/` 目录。
glim 那边唯一被用到的是 `CloudCovarianceEstimation`（搬过来后 302 行，含说明注释），已整文件搬进 `src/`。

---

## 依赖

版本以本机实测为准（`ldd` + 各自的 CMakeCache），不是"应该能用"的推测。

| 依赖 | 版本 | 来源 / 位置 | 备注 |
|---|---|---|---|
| **GTSAM** | **4.3a0** (`3ad4b4c3c`) | [borglab/gtsam](https://github.com/borglab/gtsam) —— **用 build tree, 未安装**<br>`/mnt/nvme0n1p2/gtsam_map/gtsam/build` | 见下方"两个坑" |
| **gtsam_points** | **v1.2.2** (`9d32e7d`) | [koide3/gtsam_points](https://github.com/koide3/gtsam_points)<br>工作树 `gtsam_map/gtsam_points_glim`<br>装到 `gtsam_map/install/gtsam_points-1.2.2` | `BUILD_WITH_CUDA=OFF`，只用 CPU 那套 |
| glim | `184e677` | [koide3/glim](https://github.com/koide3/glim) | **只借了一个文件**（见下） |
| Eigen | 3.3.7 | `/usr/local/include/eigen3` | GTSAM 是 `GTSAM_USE_SYSTEM_EIGEN=ON` 编的，必须和它同一份 |
| Ceres | 2.0.0 | `/usr/local` | `submap_ceres.hpp` 的局部 BA |
| OpenCV | 4.6.0 | `/usr/local/lib` | core / imgproc / imgcodecs / features2d / calib3d |
| Boost | 1.74.0 | 系统 | serialization / filesystem |
| yaml-cpp | 0.7.0 | 系统 (`libyaml-cpp0.7`) | 参数体系（§3） |
| zlib | 1.2.11 | 系统 | `png_io.hpp` 压 PNG 的 IDAT |
| nlohmann/json | 3.10.5 | 系统 (`nlohmann-json3-dev`) | 读 manifest / egomotions / 3dod 标注 |
| TBB | **libtbb.so.2 来自 `libtbb2` 2020.3**<br>libtbbmalloc.so.2 来自 `libtbbmalloc2` 2021.5.0 | 系统 | 见下方"两个坑" |
| OpenMP | 编译器自带 | — | 各处 `#pragma omp parallel for` |

### glim 只借了一个文件

`src/cloud_covariance_estimation.{hpp,cpp}` 从 glim 原样搬来（`include/glim/common/` +
`src/glim/common/`），只改三处：namespace `glim` → `ialign`、include 改相对路径、
两处 `spdlog::critical` 换成 `fprintf`（去掉 spdlog 依赖）。

**没有换成 gtsam_points 自带的 `covariance_estimation`**，虽然它就在那儿：glim 这个用
`RegularizationMethod::PLANE` 把最小特征值抬起来，平面上的点因此得到"扁"协方差——GICP 的
plane-to-plane 行为完全来自它。换实现会改变所有配准数值，本文档里的实测结论就都不可比了。

其余的 glim 依赖已全部去掉：不需要 libglim.so、不需要先编译 glim、不需要 glim 的 `config/`
目录（那几个 json 只被 glim 的 `GlobalMapping`/`IMUIntegration` 读，本程序两个都不用），
也不需要 `GLIM_USE_OPENCV` 宏（那是为了和 libglim.so 的虚表布局一致）。

### 两个坑

**GTSAM 用的是 build tree，不是安装。** `/usr/local` 里还装着一个旧的 GTSAM 4.1.1，
`find_package(GTSAM 4.2)` 会命中它然后报 "not compatible with requested version"。所以
`CMakeLists.txt` 在 `find_package` **之前**就把路径塞进 `CMAKE_PREFIX_PATH`：

```cmake
set(GTSAM_BUILD_DIR "/mnt/nvme0n1p2/gtsam_map/gtsam/build" CACHE PATH "GTSAM 4.3a0 build tree")
set(GTSAM_POINTS_PREFIX "/mnt/nvme0n1p2/gtsam_map/install/gtsam_points-1.2.2" CACHE PATH "...")
list(PREPEND CMAKE_PREFIX_PATH "${GTSAM_BUILD_DIR}" "${GTSAM_POINTS_PREFIX}")
```

换机器只改这两行。曾经因为这段缺失（当时只在 RPATH 里用了这个路径、没参与查找），删掉 build
目录重配就直接失败——之前一直能过是靠 `CMakeCache.txt` 里缓存的 `GTSAM_DIR`。

运行期还有一层：CMake 默认生成 `DT_RUNPATH`，而 **RUNPATH 不传递**——`libgtsam_points.so`
自己没有 RUNPATH，加载器解析它的 `libgtsam.so.4` 依赖时不参考可执行文件的 RUNPATH，于是
退回 ldconfig 缓存、命中 4.1.1，表现为运行期 `undefined symbol: gtsam::NonlinearFactor::error(HybridValues)`。
所以用 `-Wl,--disable-new-dtags` 改生成 `DT_RPATH`（可传递），并把本地 4.3a0 放最前。

**TBB 有两套 ABI 装在同一台机器上。** 本机 GTSAM 是 `GTSAM_WITH_TBB=ON` +
`GTSAM_DEFAULT_ALLOCATOR=TBB` 编的，于是它的头文件把 `NonlinearFactorGraph` 的底层容器换成
`tbb::tbb_allocator`，**可执行文件必须自己链 TBB**。而系统里：

```
libtbb.so.12  (libtbb12 2021.5.0, 也是 libtbb.so 的指向)  -> 缺 deallocate_via_handler_v3 等符号
libtbb.so.2   (libtbb2  2020.3)                           -> GTSAM 实际需要的这套
```

所以不能简单写 `-ltbb`，`CMakeLists.txt` 里用 `find_library` 显式指到 `.so.2`。

---

## 1. 增量的含义

已优化的 session 的**位姿和约束都存档**，下次直接复用不重算：

```
=== session 清单 (存档目录 .../store_incr5) ===
  [0] WL_CG7797_clip_20260713_jjst2  帧=2500  **已优化, 本次跳过**
  [1] WL_CG7797_clip_20260713_jjst3  帧=1697  **已优化, 本次跳过**
  [2] WL_CG7797_clip_20260713_jjst4  帧=1796  待优化 (无存档)
```

"跳过"指的是**跳过它自身的分窗 BA 和回环**，不是把它排除在优化之外。它优化后的位姿会读回
内存，之后所有路径（拼 target、建约束、导出）读到的都是**优化后**的值，并且仍然参与最终的
联合优化。

**为什么不把老 session 钉死**（`freeze_prev 0`）：钉死会让新 session 只能单方面往老的上凑，
实测 jjst2↔jjst3 的一致性从 0.083 退化到 0.095。放开几乎不要钱——稀疏图解一次是秒级，
成本全在配准。

代价是老 session 的位姿会被新数据微调。这是**符合预期的**：某个区域原本只有两个 session
的少量帧，后来又来了很多帧，数据越多整体越准。而每帧都有 INS 先验兜着，后面的数据再多也
不会把老位姿拉跑（实测 5 session 联合时各 session 位移中位 0.007–0.016 m）。

### 存档的复用判据

1. `<session>.csv` 存在
2. 头部的 `base_utm` 对得上
3. 帧集**按 pcd 路径 100% 命中**

第 3 条是硬要求，帧集一变就当作待优化而不是静默错位。注意 `--load_roi` 会改变帧集，所以带
ROI 和不带 ROI 的两次运行不能共用一个 `--pose_store`。

---

## 2. 处理流程

```
加载 session (manifest / egomotions / clips)
      │
      ├─ 已有存档 ──> 读回优化后的位姿, 跳到最终联合优化
      │
      └─ 待优化
           ├─ buildIntra   分窗 BA: 窗口 40 帧、步长 20 (50% 重叠), 窗口内建帧间 GICP 约束
           ├─ buildLoop    回环: 只按距离+里程收候选, submap2submap + 局部 BA
           ├─ buildCross   跨 session: 沿共位段每 region_step 开一个区域,
           │                目标端联合 BA -> 拼 target -> 源端 submap 逐个配上去
           └─ 写存档 (位姿 csv + 约束 csv)
      │
最终联合优化: 全部 session 的位姿一起解 (约束从存档复用, 每帧一条 INS 先验, 只锚第一帧)
      │
写 g2o / 约束图 / 存档, 然后导出点云
```

### 三类约束

| kind | 来源 | σ | 说明 |
|---|---|---|---|
| 0 | 窗口内帧间 GICP | `ba_rel_t/r` | Δ ∈ `nms_win_deltas` |
| 2/3 | 同 session 回环 | `loop_sigma_t/r` × `loop_anti_w` | 现在一律按 3（见 §4.1） |
| 1 | 跨 session scan2submap | `cross_sigma_xy/r` | 含 `cross_ba` 产出的 ref↔ref 边 |

加上每个位姿一条 `EDGE_SE3_PRIOR`（INS 位置 + 重力姿态先验）。

---

## 3. 参数体系

**全部 122 个参数由 yaml 指定**，命令行选项仍可用且**在 yaml 之后应用 = 覆盖**：

```bash
./build/local_align --params params/main.yaml --region_step 50 --cross_ba 1
```

每次运行把**生效后**的参数写到两处：

```
<out>/params_effective.yaml        这一次实际用的
<pose_store>/params.yaml           造出这批存档所用的
```

写"生效后"而不是复制输入文件——有命令行覆盖时，只有生效值才是真正用的。

### 为什么必须这样

增量建图的要害是**前后两次参数必须完全一致**，而存档的复用判据只看帧集是否命中，
**改配准参数不会让存档失效**。靠命令行传就很容易把两套参数配出来的约束混进同一张图而
毫无提示。所以每次运行会与存档里那份逐项比对：

```
!! 与存档相比, **影响已存档约束**的参数变了 11 项:
   cross_ba          存档 1  -> 本次 0
   nms_win_deltas    存档 "1,5,20,40" -> 本次 ""
   ...
```

差异分两类：影响已存档约束的（危险）和只影响本次求解/导出的（安全）。配准类参数有差异时
**不覆盖**存档的 `params.yaml`，另存 `params_conflict.yaml`——否则新参数盖上去就是在撒谎，
而且下次就再也比不出差异，警告只响一次。

**下次增量直接用存档里那份**，前后参数必然一致：

```bash
./build/local_align --params <上次的 pose_store>/params.yaml --out <新目录>
```

### 已废弃的参数

删掉的参数进 `kRetired` 名单：`render_sample_pairs`、`config`、`loop_dyaw`、`loop_bidir`、
`loop_any_dyaw`。旧存档的 `params.yaml` 带着这些键时会打印"已废弃，忽略"而不是硬失败——
未知键是硬失败（防拼错），但已废弃的键不该让旧存档整个读不进来。

---

## 4. 关键设计决策（都有实测依据）

### 4.1 回环不判航向

原来有两个门：同向 `dd ≤ loop_dyaw`(30°)、反向 `dd ≥ 180 − loop_dyaw`(150°)，
**中间 30°–150° 是死区**。十字路口垂直经过恰好落在里面。

实测 jjst2 在 (14, −450) 那个路口：两次经过（帧 406–487 航向 −116°，帧 1728–1788 航向 +148°）
**距离最近的一对是 471↔1744，相距仅 2.08 m，航向差 127°**——被死区排除。建成的边全在路口
两侧（航向差已 ≥150°）。也就是最该建约束的那一对一直没建。

关键帧有 INS 初值，配准有好起点，航向门只是多余的保险。去掉后实测：

```
交叉边 (dyaw 90~150°)  14 条  inlier 中位 0.713  100% 采纳
反向边 (dyaw ≥150°)   703 条  inlier 中位 0.791  100% 采纳

  边          dyaw    inlier   nn_before → nn_after
  472-1741   133.0°   0.819    0.3499 → 0.1225
  469-1743   114.9°   0.773    0.3187 → 0.1418
  468-1746   110.4°   0.625    0.3218 → 0.2052
```

**一律用保守那一档**（`loop_min_inlier_rev` 0.45 的门 + `loop_anti_w` 0.5 的权重），不再按
航向分档。理由：有里程门之后"同向重访"几乎不出现——实测 jjst2 的 320 条回环边里**同向 0 条**，
因为空间上挨着又同向的帧对全是红灯停车的零基线对，已被 `loop_min_arc 50` 滤掉。而交叉边
用同向的 0.6 门会拒掉一部分（实测范围 0.554–0.819）。

三个参数（`loop_dyaw`/`loop_bidir`/`loop_any_dyaw`）随之删除。

### 4.2 里程门 `loop_min_arc`

序号差门（`|i−j| > intra_win`）拦不住红灯停车。实测"空间 <12 m 且序号差 >40"的帧对里，
91.4% 是"同向"，但它们的弧长间隔中位只有 **1 m**——全是停在红灯前的帧。这类对没有基线，
给不出任何信息，却会把 `loop_step`/`loop_max` 的预算吃光。

只有弧长门能拦住它：候选链 `3555 → 弧长门(≥50m) 1861 → 抽稀 717 → 四维NMS 320`。

### 4.3 四维 NMS

抑制边 (i,j) 的条件是：存在已接受的 (i',j') 使 **|Pi−Pi'| ≤ R 且 |Pj−Pj'| ≤ R**（两端都近才
算重复）。用中点做 key 是有损的——不同的重访会算成同一处。

按 (类别, session 对) 分组，所以 A↔B 永远不会抑制 A↔C，多 session 互相印证不会被去重吃掉。

实测效果（jjst4 的回环）：50 条 → 23 条，相邻边弧长间隔中位从 **0.0 m**（全是零基线对）
变成 3.8 m，`<0.5 m` 占比 51% → 27%，单格最多 22 → 11。跨 session 边单格最多 129 → 12，
而格子数和中位数不变——削的是堆积，不是覆盖。

### 4.4 `nms_win_deltas`

窗口内**只建**指定帧号差的约束，且**在候选阶段就拦**，所以省的是配准时间（成本大头）。

`"1,5,20,40"`：Δ=1 是链，Δ=5/20 是跨越式直连。不能只给 "1"——链 σ=0.035 串 40 帧累计
0.22 m，而一条 Δ=20 的直连边把两端锁在 0.035。实测 jjst2 的窗口边分布：Δ=1 有 2493 条
（≈帧数，就是那条链）、Δ=5 有 1387 条、Δ=20 有 1059 条。

⚠️ **`40` 这一档从未生效**：`i,j ∈ [0,m)` 而 `m ≤ intra_win = 40`，所以 `j−i ≤ 39`。
所有实测数据其实是在 `{1,5,20}` 下测的。要让最大跨度生效得写 39，或把 `intra_win` 提到 41+。

### 4.5 `cross_ba`

跨 session 配准的 target 是多个 session 的帧拼起来的，而它们之间本来就不一致（离线量到
0.064–0.139 m），拼出来的 target 自带重影，配准精度被它封顶。

做法：目标端的参照帧先做一次**联合局部 BA** 再拼。实测 138 个区域，BA 挪动中位 **0.124 m**
（正是那个不一致的量级），target 自身重影 **0.10–0.14 → 0.058 m**，配准后区域内 nn 0.060。

**ref↔ref 边不是可选的**：联合 BA 把 A 和 B 相对挪动了，target 处在一个新的局部规范里。
C 配上去得到的位姿在那个规范里，若拿存档的参考位姿表达这条边，就错了整整一个 A–B 不一致
（0.07–0.14 m）。所以必须把 BA 产出的 ref↔ref 相对位姿也发成约束（`cross_ba_edges 1`）。

### 4.6 σ 的标定：rms≈1 不等于几何正确

用白化残差 rms 自校准 σ（`--calib_rounds`）**只在有冗余时有意义**：

| 约束类 | 残差数/参数数 | 是否可信 |
|---|---|---|
| 窗口边 | 19791/10664 = 1.9 | 可信，一轮收敛 |
| 跨 session | 2886/10664 = 0.27 | 有偏，永不收敛 |
| INS 先验 | 10664/10664 ≈ 1 | **完全自证**（故 `calib_priors` 默认 0） |

实测开了 `calib_rounds` 反而更差：rms 全部干净收敛到 1.00，但跨 session 一致性中位从
0.071 涨到 0.086、最差格 0.168 → 0.282。rms≈1 只证明网络自洽，不证明几何正确——长链累积
漂移没有任何残差能测到。

`calib_priors 1` 会发散：INS σ 从 0.150 一路掉到 0.005，位姿被钉死。

### 4.7 加大跨 session 权重没有用

σ_cross 扫描（其余不变，复用存档只重解）：

| σ_cross | 0.090 | 0.030 | 0.015 | 0.008 |
|---|---|---|---|---|
| session 间 nn | **0.0900** | 0.0912 | 0.0911 | 0.0910 |

126 倍权重把残差从 3.4 cm 压到 <1.8 cm，而实际一致性**反而略差**。收紧一个软方向只是把
误差推给先验（INS rms 0.223 → 0.275，姿态 0.155 → 0.254）。所以保持 `cross_sigma_xy 0.09`。

配准精度的真正上限是 **target 自身重影**：`cross_ba` 之后是 0.058 m，而配准后区域内 nn
是 0.060——已经贴着上限，瓶颈在 target 清晰度，不在权重。

### 4.8 其他实测标定

| 参数 | 值 | 依据 |
|---|---|---|
| `ba_rel_t` | 0.035 | 关键帧↔关键帧的配准可重复性。之前沿用的 0.20 是 submap↔submap 的数，大了 6 倍 |
| `ba_ins_xy` | 0.15 | 抵抗整图共模漂移的唯一力量。信息量对抗：先验 10610 条 × 1/0.15² = 4.7e5 vs 跨 session 3103 条 × 1/0.09² = 3.8e5 |
| `ba_att_w` | 275 | 原来的 50 偏轻 5.5 倍 |
| `region_step` | 50 | vs 100：区域 9→16，跨 session 边 161→211，区域内 nn 0.075→0.058，耗时仅 +12% |
| `frame_voxel` | 0.05 | 0.03 让厚度 −25% 但耗时 +185%；0.10 两项都变差 |
| `tgt_voxel` | 0.90 | **必须 ≥ 3× max(frame_voxel, submap_voxel)**，否则 inlier 塌到 0.2 量级、帧全被剔，而日志看不出异常 |
| `opt_ext` | 1 | 与 yaml 初值差 1.22°；**外参必须跟着 g2o 走**，漏了它下一轮老 session 会被挪 3.56 m |
| `z_above` | 0.5 | nn 一律只用地面以上的点。全部点的最近邻对路面沿面滑动，实测低估约一倍 |
| `ba_visual` | 0 | 两条独立口径都是零收益（一致性 0.069 vs 0.071，厚度 0.0705 vs 0.0704），代价 5 倍耗时 |

---

## 5. 踩过的坑

### 5.1 改配准参数不会让存档失效

见 §3。这是设计上的必然（复用判据只看帧集），所以只能靠 `diffParams` 提示。曾经差点用
`main.yaml`（改进前的基线，11 项配准参数不同）去接一个用新参数造的存档。

### 5.2 g2o 残差的坐标系

`VERTEX_SE3:QUAT` 是 `T_w_v`（车体位姿），而 `EDGE_SE3:QUAT` 是**雷达系**相对位姿 `T_i_j`。
直接用顶点算相对位姿去比测量，会差一个外参共轭，残差全部落在 5.29–5.50 m（≈ 2×|t_vl|，
外参平移 3.302 m）。正确写法：

```
残差 = log( Tm⁻¹ · (T_v_l⁻¹ · T_w_v_i⁻¹ · T_w_v_j · T_v_l) )
```

修正后回环残差中位 0.058 m，与另一条独立算法一致（0.0606）。

**分布过窄就是算错了的信号**：真实残差不可能全部挤在 0.2 m 的区间里。

### 5.3 共享端点才能比较两条边的矛盾量

`T(i,j1)⁻¹ · T(i,j2) = T(j1,j2)` 只在**共享 i** 时成立。若允许两端各差几帧，那几帧的车程
（5 m/帧 × 6 帧 = 30 m）会全算进"矛盾量"，得出 11.8 m 这种荒谬的数。

### 5.4 窗口重叠导致的重复配准

`buildIntra` 的去重原本在**配准之后**（配完再按 (i,j) 丢弃重复），而窗口 50% 重叠，重叠区
的对会在两个窗口各配一次 VGICP，第二次的结果算完就扔。

**解决**：用初值在候选阶段就跨窗口查重。实测 GICP 候选 **601 → 358（−40%）**，跳过 243 对，
采纳数不变，所有输出逐字节相同。保留配准后的去重作为**自检**（一次遍历很便宜，而"重复约束
等于把同一个测量的权重加倍"这种错很难从结果看出来）。

之所以能安全提前：同一对在两个窗口里的初值、点云、体素图分辨率完全相同，那几道门和 VGICP
都是确定性的——第一个窗口过不了的，第二个窗口同样过不了。这一点由"输出逐字节相同"确认。

### 5.5 N 条边共享一个噪声实现

`--cross_per_sess` 让一次配准派生出 N 条边（对每个之前的 session 各一条）。它们共享同一个
噪声实现，直接给每条独立的 σ 等于把那次测量的权重算了 N 遍。所以 σ 要 ×√N，通过
`CalibIn::sig_scale` 传递并存进存档的额外一列。

### 5.6 UTM 带号无法从坐标推断

一个点相对自己带的中央经线的偏移总在 ±3° 内，所以**任何带号都自洽**——只凭 (easting,
northing) 数学上定不了带号，猜错会把图放到地球上错误的位置且不报错。所以 `--utm_zone`
默认 0（不写 CRS 声明，坐标照样是绝对 UTM）。参考代码里那个自动推断的实现有两个 bug
（用了带西边界而不是中央经线；判据对 easting < 500000 永不成立），实际总是回落到硬编码值。

---

## 6. 重影排查：一次尚未收敛的调查

现象：jjst2 单 session 内部，路口的导流线在 rgb 点云里有重影（强度点云里看不清）。

**已经排除的**：

| 假设 | 实测 | 结论 |
|---|---|---|
| 回环把位姿拽歪了 | 去掉全部回环后，路口两腿的回环残差 0.019 → **0.102 m** | 回环本来就在把两腿拉到 2 cm，去掉更差 |
| 航向死区漏了约束 | 补上 5 条交叉边后位姿只动 **4.7 cm** | 漏洞是真的，但不解释重影 |
| 全局约束在拉这个路口 | 局部单独重跑 205 帧（无长链、无远处约束），与全局解差 **1.7 cm** | 不是长链累积 |
| 上色整体偏移 | 颜色 vs 强度的 2D 互相关，14 块位移中位 **0.000 m** | 没有系统性偏移 |
| 垂直方向 | 地面点沿平面法向展宽（p5–p95）中位 **2.3 cm** | z 方向对得很好 |

**尚未验证的**：颜色的**逐帧/分腿**偏移。如果第一腿偏 +0.15 m、第二腿偏 −0.15 m（例如时间
同步误差沿运动方向，而两腿反向），平均位置与强度一致（互相关为 0），却会形成 0.3 m 间距的
双线——这恰好被互相关掩盖，也能解释"rgb 有、强度没有"。

**缺的关键信息**：重影两条线的间距。0.02–0.05 m 是点云本身的厚度下限（不是重影）；
0.1–0.2 m 要找是哪两段错开；0.3 m 以上则与位姿图无关。

**建议的下一步**：加 `--export_frames a,b`（按帧号导出），把单帧、单腿分别导出，直接量出
"单帧内部 → 帧间 → 腿间"各占多少厚度，不必依赖在图上找线。

---

## 7. 性能

### 实测耗时

| 场景 | 帧数 | 耗时 | 峰值内存 |
|---|---|---|---|
| 4 session 从零（含 las 导出） | 8008 | 2h58m | 9.2 GB |
| jjst2 单跑（配准+优化 / 导出） | 2500 | 19m17s / 20m50s | — |
| jjst2 纯导出（复用存档，无 las） | 2500 | 1m38s | — |
| ROI 单跑（205 帧，完整流程） | 205 | 1m07s | 4.1 GB |

峰值内存**不在配准阶段**（那里只有 1.4 GB），而在导出——`_after.pcd` 要对全部点做一次全局
体素滤波，所以得把所有帧收进内存。流式写盘那一路（las/rgb）反而不占。

### 三项 IO 优化（均验证输出逐字节相同）

| | 效果 |
|---|---|
| 帧缓存（LRU，按字节限容） | 命中 **83%**，耗时 −12% |
| 导出一次读派生两份 | 导出阶段 IO 减半 |
| 候选阶段去重 | GICP 候选 **−40%** |

**关于帧缓存的收益，一个被修正的判断**：原本按"每帧被读 12–16 次 × 20 ms = IO 占七成"
估计能省 40–60%，实测只有 12%。原因是 **Linux page cache 已经吸收了磁盘层的重复读**
（系统 buff/cache 有 27 GB），那 20 ms 里绝大部分是 CPU：lzf 解压 + range 过滤 + 读 3dod
标注剔动态 + 体素滤波。帧缓存省的是这部分 CPU。

单帧缓存单价：`frame_voxel 0.05` 时约 24678 点/帧 × 40 B（`Vector4d` 32 B + 强度 8 B）
≈ **1 MB/帧**；0.03 时约 2.5 MB/帧。所以 `--frame_cache_mb` 按**字节**限容而不是帧数。
导出阶段会主动清空缓存（那里每帧只顺序读一次，命中率为零，留着只跟那 10 GB 累积争内存）。

### 还没做的

- **`Vector4d` → `Vector4f`**：内存带宽和缓存占用减半，但 gtsam_points 接口要 `Vector4d`，
  转换成本可能吃掉收益，得先测。
- **导出写盘是单线程**：帧级读取并行，但 `PcdStream::add`/`LasStream::add` 里的坐标量化和
  memcpy 串行。1.6 亿点 × 26 B，如果瓶颈在量化可以并行准备缓冲区；如果在磁盘顺序写就没得优化。
- **[local_align.cpp:4087](src/local_align.cpp#L4087) 缺 `num_threads`**：连通域分析里的
  O(n²) 距离计算用的是 OpenMP 默认线程数，其他 20 处都限制到了 `o.num_threads`。jjst5
  4671 帧 → 2180 万次距离。

---

## 8. 目录约定

```
<pose_store>/                 增量的全部状态, 几十 MB, **绝不清**
  <session>.csv               每帧一行 T_w_v (头部记 base_utm / 外参 / 帧数)
  <session>_edges.csv         约束存档 (kind, from_pcd, to_pcd, 相对位姿, nn, sigma_scale)
  params.yaml                 造出这批存档所用的参数
  pose_graph.g2o              全部位姿(优化后) + 全部约束 + INS 先验 + FIX
  pose_graph_index.csv        顶点 id -> pcd 路径 (标准 g2o 没地方放它)
  constraint_graph.png        约束图 ("约束在哪儿"比"有多少条"重要)
  cross_edges.csv             每条跨 session 边一行, 按 nn 排序找配坏的地方

<out>/                        点云/las/dump, 几十 GB, 随便清
  <session>_after.pcd         体素 0.05, 量重影/厚度的基准
  <session>_after.las         **原始点**, 绝对 UTM, 强度+RGB 在一个文件里
  <session>_after_rgb.pcd     **原始点**, 带颜色
  <session>_poses_cmp.csv     每帧 INS/优化后位姿、位移、到最近同 session 帧距离
  params_effective.yaml       这一次实际用的参数
```

`run_incr.sh` 负责"清 out、保留存档、从 yaml 读路径"这套流程，并且拦住
`pose_store` 在 `out` 里面的情况（那样清 out 会连存档一起删）。

---

## 9. 已知待办

- **`ba_rel_t` 0.035 → 0.028**：最可信的校准值（冗余度 1.9，一轮收敛），但它影响每个
  session 自己的 BA，改了必须全量重做。
- **`nms_win_deltas` 的 `40` 是死值**（见 §4.4），要么改 39 要么提高 `intra_win`。
  改之前的所有实测都是在 `{1,5,20}` 下做的。
- **重影未定位**（见 §6），缺的是两条线的间距这一个数字。
- **帧读取失败的守门（`--max_io_fail`）未在真实场景验证**：外挂盘完全掉线时
  程序更早就在"没找到任何 session"退出，那个保护一次都没被调用过。要复现
  "session 已加载、跑到中途才读不到"的场景，需要临时移走一部分 pcd 再跑，
  期望 `exit 2` 且存档指纹不变。
- **`b_max_dyaw`（跨 session 的航向门）还在**。回环那边已经证明航向门是多余的，但跨 session
  是 scan-to-submap、目标端是多 session 拼的 submap，情况不同，要改得单独验证。

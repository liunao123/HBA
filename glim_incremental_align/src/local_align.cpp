// -----------------------------------------------------------------------------
// local_align —— 指定一个位置, 把 B 的帧逐一配到 A 的局部 submap 上
//
//   1) 在给定中心的半径内, 取 session A 的帧, 按各自位姿拼成一个 submap;
//   2) 取同一邻域内 session B 的帧, **每帧独立**配准到这个 submap (VGICP + GICP 精化);
//   3) 配准结果即该帧的新位姿。所有中间产物都落盘供人眼检查。
//
// 为什么单独做一个工具: 主管线(incremental_g2o)把 submap 构造、基准图优化、逐帧挂接、
// 联合优化串成一条链, 中间任何一环出问题都会被后面的环节掩盖或放大 —— 这个项目里已经
// 因此绕过好几次(f2m 拿错帧的点云、kfba_voxel 违规让 inlier 静默塌掉)。
// 这里只留最短的一条链: 拼 target -> 逐帧配准。变量少, 每一步都能单独看。
//
// 落盘的东西 (都以"分数在最前"命名, 所以 ls 天然按质量排序, ls -r 从最差看起):
//   submap_A.pcd                A 的邻域帧拼出来的 target
//   submap_A_odd/even.pcd       按帧序奇偶拆两半 —— **叠着看就是 target 自身的重影**
//                               这一项必须先看: 配准精度的上限就是 target 自身的清晰度,
//                               它糊到 0.1 就别指望把 B 对到 0.03。
//   frames/nn<配准后>_g<改善>_f<帧号>_{before,after}.pcd + .png
//   B_before.pcd / B_after.pcd  B 的全部邻域帧, 配准前/后, 拼在世界系
//   result.csv                  每帧一行: inlier / 修正量 / nn 前后 / 是否采纳
//
// 判读要点 (都是这个项目里踩出来的):
//   - nn 一律只用**地面以上**的点。全部点的最近邻对路面沿面滑动是瞎的, 实测低估约一倍。
//   - target 体素图分辨率必须 >= 3 x 点间距, 否则 inlier 会静默塌到 0.2 附近。下面有自检。
//   - 动态物体(车)必须剔掉: 一帧 0.5 秒, 10m/s 的车走 10 米。不剔的话 target 里全是车的
//     残影, 而那不是位姿误差, 配准修不掉也不该去修。
// -----------------------------------------------------------------------------

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <iomanip>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include "cloud_covariance_estimation.hpp"   // 从 glim 搬过来的那一份 (见该文件头部)
#include "pcd_io.hpp"
#include "session_loader.hpp"
#include "submap_ceres.hpp"
#include "topdown_png.hpp"
#include "visual.hpp"   // 只为了 removeDynPoints (动态物体框)

namespace fs = std::filesystem;

// -----------------------------------------------------------------------------
struct Opt {
  // --save_params <f>: 把生效后的参数写成 yaml 然后退出 (生成初版模板用)
  std::string save_params;
  std::string root = "/media/tyjt/Elements/WG_wuling/incremental_mapping";
  std::string out = "./out_local";
  std::string data_mode = "clips";
  // std::string pose_dir = "localized";
  std::string pose_dir = "egomotions";
  std::string attitude_from = "quat";
  std::string lidar_dir = "fuse_lidar";
  std::string pose_src = "sparse";
  int sa = 0, sb = 1;              // A = 提供 submap 的 session; B = 要优化的 session
  double cx = 0, cy = 0;           // 邻域中心 (与日志里的轨迹包围盒同一坐标系)
  double radius = 30.0;            // 邻域半径 (m): A 取 submap, B 取待配准帧
  double b_radius = 0.0;           // B 的半径 (<=0 则同 radius)
  // ---- 按"B 帧位姿 vs 拼 submap 用到的 A 关键帧位姿"的关系筛 ----
  // 只在中心半径内选 B 帧是不够的: 离 A 的帧太远的 B 帧, 看的是 A 没覆盖的地方,
  // 实测 85% 的源点会被覆盖门剔掉, 剩下的样本又少又偏, 配准自然不稳。
  // 航向也要看: 同一位置反向行驶时, 杆子被照射的是相反那一面、近密远疏的采样分布对调、
  // 遮挡模式相反 —— 残差里混着配准消除不掉的系统性成分, GICP 会把它当位姿误差去"修"。
  double b_max_dist = 10.0;         // B 帧到最近的 A 关键帧的距离上限 (m); <=0 关闭
  // 默认**关闭**: 这是距离判据之外的额外筛选, 开着会静默丢帧。分布照样打印,
  // 要用再显式给 --b_max_dyaw 30。
  double b_max_dyaw = 30.0;       // 航向差上限 (度); >=180 关闭
  // 1 = 跨 session 也收**反向**共位 (航向差 >= 180 - b_max_dyaw)。
  // 为什么安全: 航向门只决定"这帧算不算共位、边挂到哪个参照帧", **不影响配准看到什么**
  // —— submap 的组成 (ra) 只按距离筛, 里面本来就有两个方向的点。
  // [实测 incr4] 路口 60m 内各 session 的航向是 -33.5 或 +146 (同一条路的两个方向):
  // jjst3/jjst5 走一个方向, jjst4 走反方向 -> jjst4<->jjst3 和 jjst4<->jjst5 在那个路口
  // **一条约束都建不出来**, 只能各自绕 jjst2。这就是斑马线在路口不一致的直接原因。
  // 另有 jjst5 的 376 帧 (15.5%) 是 10m 内有参照帧但没有同向的, 整帧被丢。
  int b_bidir = 0;
  // 1 = 每个被并入帧对**每个**之前的 session 各建一条边 (取该 session 里最近的合格帧),
  // 而不是只挂到全局最近的那一个。
  // [实测 incr4] jjst5 有 799 帧同时被 2 个之前的 session 覆盖, 但每帧只挂了 1 条 ->
  // 漏掉 924 条 (比实建的 1920 条多 48%)。submap 里用了那些点, 图里却没有对应的边。
  // **注意**: 同一次配准派生的 N 条边不是独立测量 (共享同一个配准噪声), 所以它们的
  // sigma 会自动乘 sqrt(N) —— 否则等于把那次测量的权重算了 N 遍, 正是我们在去重时
  // 要消除的那种虚高。
  int cross_per_sess = 0;
  // ---- 跨 session 配准: 两端都先做局部 BA, 再 submap2submap ----
  // 1 = 目标端(邻域内前面所有 session 的帧)先做一次**联合**局部 BA 再拼 submap。
  // 为什么: 混合 submap 把 A、B 在这个位置的不一致直接烙进 target, 而配准精度的上限就是
  // target 自身的清晰度。[实测] 6 处三 session 覆盖的位置, 两两 nn 是 0.064~0.139 —— 而
  // 现在跨 session 配准后的 nn 就是 0.07~0.10, **已经贴在上限上了**。
  // **联带效应(必须处理)**: 联合 BA 会把 A、B 相对挪动, target 于是处在一个新的局部规范系,
  // 而图里 ref 的位姿还是存档值。所以必须把 BA 产出的**跨session ref<->ref 相对位姿也发成
  // 约束**(cross_ba_edges), 否则 C 的边和图里的 ref 位姿不在同一个系里, 错量正好是 A-B 的
  // 不一致。这同时也就产出了"同一位置 AB/AC/BC 三条约束"。
  int cross_ba = 0;
  int cross_ba_edges = 1;   // 1 = 目标端 BA 产出的跨session ref<->ref 相对位姿也发成约束
  // 目标端做 BA 的帧数上限。region_max_frames(150) 那些帧同时带协方差+体素图约 1.2GB,
  // 这个项目被 OOM 杀过四次, 所以 BA 这一路单独封低一点。
  int cross_ba_max = 40;
  // >0 = 源端也取当前帧 +-n 帧拼 submap (先局部 BA), 做 submap2submap 而不是 scan2submap。
  // 单帧配到 submap 上时源侧只有一次扫描的采样, 反向/斜交时共同可见面小; 拼一小段之后
  // 共同表面大得多 (回环那边实测 inlier 0.585->0.790, nn 0.204->0.119)。
  int cross_src_win = 5;

  double frame_voxel = 0.05;
  double submap_voxel = 0.05;      // submap 拼好后的体素 (= target 的点间距)
  double min_range = 5.0, max_range = 40.0;
  double tgt_voxel = 0.9 ; //  submap_voxel * 6.0;       // target 体素图分辨率, 必须 >= 3 x 点间距
  int iters = 20;
  bool fine = true;
  double fine_corr = 0.5;
  int fine_iters = 15;
  double min_inlier = 0.4;
  double max_corr = 2.0;           // 水平修正上限 (m), 超了不采纳
  double z_above = 0.5;            // 地面以上多少米算竖直结构
  bool clamp_planar = true;        // 只采纳 yaw+x,y; z/roll/pitch 取初值
  bool dyn_filter = true;          // 剔掉动态物体上的点 如果存在3dbox
  double dyn_ts_tol = 0.02;
  int dump_max = 200;              // 最多存多少帧的前后点云+图
  // 帧缓存上限 (MB)。0 = 关。见 FrameCache 的说明。
  //
  // 实测单价: frame_voxel 0.05 时约 2.5 万点/帧 = 1 MB/帧 (Vector4d 32 B + 强度 8 B);
  // 0.03 时约 6 万点/帧 = 2.5 MB/帧。所以按**字节**限容, 不按帧数。
  // 默认 1500 MB: 配准阶段本身只占 1.4 GB, 加上它到 3 GB 左右; 而 buildIntra / 回环 submap /
  // buildCross 的工作集都在几十到一百多帧, 这个容量足够覆盖。
  // 帧读取失败率上限。超了就中止且**不写存档** (见 guardIo)。<=0 = 不检查。
  // 默认 0.01: 正常情况下一帧都不该读失败, 1% 已经很宽松 —— 它防的是数据盘掉线那种
  // 整段读不到的情形(实测一次掉线就是 34566 次失败), 不是偶发的单帧损坏。
  double max_io_fail = 0.01;
  int frame_cache_mb = 1500;
  int num_threads = 6;
  // 只扫两条轨迹的共位情况然后退出 —— 选测试位置前必须先知道它们在哪里真正走了同一条路。
  bool scan_overlap = false;
  double scan_step = 150.0;        // 扫描时沿共位段每隔多少米给一个候选中心
  // ---- A 的 submap 内部 BA (Ceres: INS 先验 + 帧间 GICP + 视觉重投影) ----
  // 为什么必须做: 实测共位段上 A 的 submap 自身重影 0.176m, 而 B 帧配准前离它只有 0.120m
  // —— B 本来就比参照物还齐, 没有可修的东西。配准精度的上限是**参照物的清晰度**,
  // 所以要先把 submap 自己做齐。
  bool ba = true;
  double ba_min_overlap = 0.2;     // 帧间建 GICP 约束的最小重叠率
  double ba_pair_voxel = 0.9;      // 帧间 VGICP 的体素图分辨率
  int ba_pair_iters = 20;
  double ba_max_corr = 1.0;        // 帧间测量的修正上限 (m), 超了不要这条约束
  // ---- 以下默认值由**白化残差实测**校准而来, 不是拍的 ----
  // 校准规则: sigma_new = sigma_old x rms (以权重给的项则 w_new = w_old / rms)。
  //   rms < 1 = 实际残差比假设的噪声小 = 权重给轻了 (方向容易搞反)。
  // WG_wuling (285,-663) 半径 40m、20 帧、99 条帧间约束、98 条视觉对应, 未校准时实测:
  //      项        假设      rms      实测
  //      INS位置   0.30      1.320    0.396 m
  //      姿态      w=50      0.181    -> w 应 x5.5
  //      帧间平移  0.10      0.348    0.0348 m
  //      帧间旋转  0.01      0.187    0.00187 rad
  //      视觉      2.5 px    1.436    3.59 px
  // 其中"帧间平移 0.035m"顺带回答了一个悬着的问题: 关键帧↔关键帧、40m 邻域内的配准
  // 可重复性是 3.5cm, 而我一直沿用的 0.20m 是 **submap↔submap** 的数, 大了 6 倍 ——
  // 又一次拿错量级的参照物。
  double ba_ins_xy = 0.40;
  double ba_rel_t = 0.035, ba_rel_r = 0.002;
  double ba_att_w = 275.0, ba_huber = 1.0, ba_chi2 = 0.0;
  int ba_iters = 50;
  bool ba_visual = false;   // [实测] 零收益: 跨session 0.069 vs 0.071、墙面厚度 0.0705 vs 0.0704, 而耗时约 5 倍          // 加视觉重投影约束
  double ba_sigma_px = 3.6;        // 实测 3.59, 见上表
  std::string vis_cams;            // 用哪些相机 (空 = 全部)
  // 给雷达外参补一个 yaw 修正 (度), 右乘到 T_w_l 上: T_w_l' = T_w_l * Rz(delta)。
  // 实测 WG_wuling: yaw(T_w_l) - 实际行驶方向 = -2.19 度 (直线段, std 0.53), 而外参文件
  // 里的 yaw 只有 -0.32 度 -> 外参偏了约 -1.87 度。
  // 这个偏差会让**帧间相对位姿的平移在雷达系下被转 1.87 度**, 于是 GICP 与 INS 不一致,
  // BA 只能把整块补丁绕锚定帧转 +2 度去补 —— 每块各转一次、各自锚在不同帧上,
  // 结果是每块内部变齐而各块处于不同的旋转规范系里(锚间不一致 0.016~0.341m 就是这么来的)。
  // 所以要在**输入端修一次**, 而不是让 BA 每块修一遍。
  double yaw_fix_deg = 0.0;
  // 把雷达外参 T_v_l 也放进 BA 一起优化: 0=不动 1=只优化旋转 2=旋转+平移。
  // yaml 里的值作初值。见 submap_ceres.hpp 里 optimizeSubmapCeresExt 的注释:
  // 必须同时把待优化量从 T_w_l 换成 T_w_v + 共享 T_v_l, 否则外参不可观测。
  int opt_ext = 0;
  // ---- 导出点云上色 ----

  bool color = true;              // 用相机内外参给导出的点云上色
  std::string color_cams;          // 上色用哪些相机 (空 = 全部)
  double color_z_min = 1.0;        // 相机系最小深度 (m)
  double color_max_range = 30.0;   // 超这个距离不上色 (硬阈值; 地面实际由下面的入射角门决定)
  // 入射角门: 一个像素的图像误差在该点所在表面上造成的位移超过此值就不上色(m)。
  // 只对**接近地面**的点生效 —— 竖直面上 1 像素只对应 d/f (30m 处 0.039m), 不需要门。
  double color_smear_max = 0.10;
  double loop_min_arc = 0.0; // 回环两帧之间的**弧长**下限(m)。0 = 不设(基线行为)
  // 1 = **.las 和 _rgb.pcd 用原始点写** (只过 range + 动态剔除, 不做体素滤波)。
  // 纯 xyz 的 .pcd 不受影响, 始终是体素滤波后的 —— 它是量重影/厚度的基准, 换了就和
  // 之前所有测量不可比。
  // [实测 jjst2] range[5,40] 已经把每帧 20.4 万点砍到 6.75 万, 全量 2446 帧 = 1.69 亿点:
  //   las 4.4 GB/份 (before+after 8.8 GB), rgb pcd 0.5~2.7 GB/份 (看 color_drop)。
  // 必须流式写盘 (PcdStream/LasStream): 先攒后写要 7 GB 以上内存, 会被 OOM 杀掉。
  int export_raw = 1;
  // 0 = **不导 _before 那一份** (默认)。_before 用的是 INS 初值位姿, 只是个对照;
  // 导它要把全部帧再读一遍 + 再上一遍色 + 再写一份 las/rgb, 时间和磁盘都翻倍。
  // 要做优化前后的点云对比再打开。
  int export_before = 0;
  // 0 = **完全不导出点云**。参数扫描/校准时用: 一次全量导出约 25GB、要几十分钟, 而那些
  // 实验只看日志里的 rms 和位姿, 点云是纯浪费。
  int do_export = 1;
  int las = 0;        // 1 = 额外写 .las (LAS 1.2 fmt2: xyz + 强度 + 颜色, 绝对 UTM)
  int utm_zone = 0;   // >0 才往 las 里写 CRS(EPSG 32600+zone); 0 = 不声明, 见下面注释
  double color_ground_h = 0.5;     // 车体系高度低于此值的点算"地面点", 才走入射角门
  double color_occ_tol = 0.5;      // 遮挡门: 比该像素块最近深度深出这么多就不上色
  int color_occ_px = 4;            // 深度缓冲的像素块边长
  // 1 = 未上色的体素**不进** _rgb.pcd (留空洞, 但颜色干净); 0 = 涂深灰后也进。
  // 注意: 无论哪种, 纯 xyz 的 .pcd **始终是完整的** —— 它是量重影/厚度的基准。
  int color_drop = 1;
  // ---- 强度车道线导出 (判路面漆重影的工具) ----
  // ---- 原始点导出 (不做体素滤波, 只裁到一个方框内) ----
  // 全量原始点装不下(一帧 20 万点 x 上万帧), 所以只导一块公共区域。
  double raw_x = 0, raw_y = 0, raw_size = 0;   // raw_size > 0 才导
  // 按白化残差 rms 自校准各类约束的 sigma 的轮数 (0 = 只解一次, 不校准)
  int calib_rounds = 0;
  // 1 = 连 INS/姿态先验的 sigma 也一起校准。默认 0, 理由见 solveCalib 的注释(会发散)。
  int calib_priors = 0;
  
  // ---- 增量建图 (多 session 依次并入) ----
  bool incr = true;

  std::string pose_store;          // 位姿存档目录 (默认 <out>/poses)
  std::string redo;                // 强制重做的 session 名字, 逗号分隔
  std::string sess_order;          // session 顺序 (逗号分隔的名字); 空 = 目录序
  bool list_done = false;          // 只列清单就退出
  std::string load_g2o;            // 载入已有位姿图: 老 session 的约束直接复用, 不重新构造
  // 1 = 老 session 的位姿在最终联合优化里**钉死** (旧行为)。默认 0 = 一起优化。
  int freeze_prev = 0;
  // ---- 同 session 回环 (独立于分窗逻辑的一路) ----
  bool loop = true;                // 开回环 (--no_loop 关)
  double loop_dist = 10.0;         // 空间距离上限 (m)
  // 回环**不判航向** (曾有 loop_dyaw / loop_bidir / loop_any_dyaw 三个参数, 已删):
  // 同向门(<=30度)与反向门(>=150度)之间的死区正好吃掉十字路口的垂直经过 —— 实测 jjst2 在
  // (14,-450) 那个路口, 两次经过距离最近的一对相距仅 2.08 m、航向差 127 度, 一直没建约束。
  // 现在只按距离 + 里程收候选, 一律用 loop_min_inlier_rev 的门和 loop_anti_w 的权重。详见 buildLoop。
  int loop_min_gap = 0;            // 序号差下限; 0 = 用 intra_win (低于它由分窗逻辑覆盖)
  double loop_step = 20.0;         // 沿弧长每隔多少米开一次回环尝试
  int loop_per_frame = 2;          // 每次尝试最多留几条
  int loop_max = 6000;             // 候选总数上限
  double loop_min_inlier = 0.7;    // 回环的 inlier 门 (比窗口内严: 配错会把整段轨迹拽歪)
  double loop_min_inlier_rev = 0.6; // 反向回环的 inlier 门 (放宽反向匹配)
  // 回环用 **submap2submap** 而不是 scan2scan: 每端取 +-n 帧按当前位姿拼成局部 submap
  // 再配。0 = 单帧(原行为)。反向重访时单帧的视场重叠很低(同一根杆子只看到相反那一面),
  // 拼成一段路的 submap 之后共同可见的表面大得多。
  int loop_submap = 0;
  double loop_submap_radius = 25.0;  // 拼 submap 时邻帧到中心帧的距离上限(m)
  // 拼 submap 之前先对这 (2n+1) 帧做一次**局部 BA** (INS 先验 + 帧间 GICP), 用优化后的
  // 相对位姿再合并 —— 与窗口 BA 同一套逻辑, 只是范围小。
  // [实测] 直接用 INS 相对位姿拼, 目标端自身重影中位 0.143 m, 而这就是回环约束的精度上限。
  int loop_ba = 1;
  // 回环约束自己的 sigma。<=0 = 沿用 ba_rel_t/ba_rel_r (旧行为)。
  // 必须能单独给: 0.035 是**窗口内帧对**校准出来的(相邻几米、同向、重叠高), 回环是
  // 跨几百米弧长的另一类测量, 借用那个值等于把权重高估好几倍。
  double loop_sigma_t = 0.0, loop_sigma_r = 0.0;
  // 反向回环的**权重**相对同向回环的倍数。权重 = 1/sigma^2, 所以 0.5 倍权重
  // <=> sigma 乘 sqrt(2)。反向观测看到的是物体相反那一面、采样分布对调、遮挡相反,
  // 残差里有配准消除不掉的系统性成分, 不该和同向同权。
  double loop_anti_w = 0.5;
  int dump_loop = 0;                 // >0: 落盘前 n 个回环候选的俯视图 (0=关)
  // 1 = dump 时**也存 pcd**。默认 0 = 只存 png。
  // png 里已经有配准前/后的叠加(上下=前/后, 左右=全部点/地面以上), 判读够用; 而 pcd 是
  // 体积大头 —— 全量下 dump_cross 60 + 前 20 个区域的 submap/奇/偶 是好几 GB。
  int dump_pcd = 0;
  // ---- 约束去重 / 均匀化 (四维 NMS) ----
  // 问题(实测 incr4): 同一个 i 配到的多个 j 之间帧号差中位=1、96% <=3, 就是 A<->B 和
  // A<->B+-1 这种同一件事重复说几遍; 跨session 更狠, 一个参照帧被最多 181 条边同时钉着
  // (被并入端基本每帧一条)。后果是边挤在少数位置 —— jjst2 只有 32% 的 50m 格有回环边,
  // 而那些格里每格十几条: 密的地方权重虚高, 稀的地方没人管。
  //
  // 抑制条件 = **两端都近才算重复**: 已接受 (i',j'), 则 (i,j) 丢弃 iff
  //   |P_i - P_i'| <= R 且 |P_j - P_j'| <= R
  // 为什么不能只看中点: A<->B 与 A<->B+1 的中点只差半个帧距, 但两个几何上完全不同的对
  // 也可能共享中点 —— 中点是有损的键。实测四维比二维多留 121/458 条回环、245/1640 条
  // 跨session, 那些是**同一地点的不同重访**, 二维会错杀。
  //
  // 分组只在同一 (类别, session对) 内抑制 —— 所以 A 与 session1 的 B 建了约束后,
  // session2 里同样在 B 附近的 C 仍然会建 A<->C。多 session 互相印证不能被去重吃掉。
  // 实测(20m 格): 有跨session边的格 369->368, 连 3 个 session 的格 108->108。
  double nms_loop = 0.0;    // >0 = 回环用四维 NMS (m); <=0 = 用旧的 loop_cluster_* 中点聚类
  double nms_cross = 0.0;   // >0 = 跨session 用四维 NMS (m); <=0 = 不去重(旧行为)
  // 同 session 窗口内只建这些帧号差的约束; 空 = 全建(旧行为)。
  // 实测同session边里 Δ=1 只占 8.2%, Δ>=2 占 92% —— 后者说的是同一段局部几何重复几十遍。
  // **这一道在候选阶段就生效, 所以省掉的是配准时间(成本大头), 不只是边数。**
  // 但别无脑只给 "1": Δ=1 的链 sigma=0.035, 串 40 帧累计 0.035*sqrt(40)=0.22m, 而一条
  // Δ=40 的直连边只有 0.035 —— 中等 Δ 的边抑制的是链上的累计漂移, 不是纯冗余。
  // 建议 "1,5,20,40" 这种对数梯度。
  std::string nms_win_deltas;
  double loop_cluster_dist = 2.0;  // 回环聚类半径 (m): 同一地点只保留少数匹配
  int loop_cluster_max = 1;        // 每个聚类最多保留多少条回环 (默认 1)
  // 每次 scan-to-submap 都把 target/源帧/配准前后 存成俯视图+pcd, 最多这么多帧 (0=关)。
  // 默认关: 全量下 900+ 次配准 x (2 份帧 pcd + png) + 每个区域一份 submap 要 1GB 以上。
  int dump_cross = 500;
  // 丢掉**孤立帧**: 半径内没有任何**同 session** 的其他帧。这种帧拿不到帧间约束,
  // 位姿只剩 INS 先验撑着, 放进优化里是纯噪声。
  // 默认 10m 而不是 5m —— 实测(WG_wuling 三个 session)正常关键帧到最近同 session 帧的
  // 距离中位就有 2.5~3.7m、最大 6.8m(车速快的路段 0.5s 一帧就这么远), 5m 会删掉
  // 14~33% 的**正常**帧; 8m 以上才一帧都删不到。阈值必须落在正常间距分布之上,
  // 否则删的是数据不是离群点。筛之前一律先把分布打出来。
  double drop_isolated = 10.0;
  // 一个连通域自身的空间尺度小于这个值 = 车在这一块里没动过 -> 整块丢掉。
  double drop_static = 5.0;
  // **默认 0 = 一帧都不丢**。连通域/静止块分析照常做并打印"本该丢哪些", 但所有帧都参与
  // 优化 —— 分析只作为诊断, 不动数据。要真丢给 --drop_frames 1。
  // 要连分析都不做, 给 --drop_isolated 0 (那条在连通域之前就退出)。
  // [实测] 丢掉的那一块 (jjst2 的 clip _000000, 54 帧原地不动、离主图 118.9m) 其实被
  // jjst5 以 3.5m 的距离覆盖过, 而 jjst5 是行驶中采的、拿得到真实帧间约束。留着它等于在
  // **已被正确建图的位置**叠一份只有 INS 精度、且无人校正的副本 —— 那正是重影的配方。
  int drop_frames = 0;

  // ---- 旧路径: 两 session 一次性联合 (已被 --incr 取代, 只留作对照实验) ----
  // !! 下面 joint 之后那些 (load_roi / intra_* / region_* / cross_sigma_*) **不是**
  //    joint 专属的 —— --incr 全都在用。只有 joint 这一个开关是选那条旧路径的。
  // A、B 各自先做整段 BA, 再遍历所有重叠区域建跨 session 约束, 最后一起解一次。
  bool joint = false;
  double load_roi_x = 0, load_roi_y = 0, load_roi_size = 0;  // 先在小范围试跑
  double intra_pair_dist = 12.0;   // 同 session 内建 GICP 约束的最大帧间距离
  int intra_win = 40;              // 分窗处理: 窗口帧数 (窗口间 50% 重叠)
  double region_step = 50.0;      // 沿共位段每隔多少米开一个重叠区域
  double region_radius = 50.0;     // 每个重叠区域的半径 (A 取这个范围内的帧拼 submap)
  // 每个区域拼 submap 最多用多少参照帧 (0=不限)。多 session 之后必须封顶, 见 buildCross。
  int region_max_frames = 150;
  double cross_sigma_xy = 0.09;    // 跨 session 约束的 sigma。**实测值**: 六处锚点的
                                   // 配准后 nn 是 0.070~0.103, 所以 0.09 是它的真实精度。
  double cross_sigma_r = 0.005;
};


// -----------------------------------------------------------------------------
// 参数的 yaml 化
//
// 为什么: 这个程序有 123 个参数, 靠命令行传等于每次跑都要重打一长串, 而增量建图的要害是
// **前后两次的参数必须完全一致** —— 存档的复用判据只看"帧集是否 100% 命中", 改配准参数
// (frame_voxel / region_step / cross_ba / intra_pair_dist ...) **不会**让存档失效, 于是
// 很容易把两套参数配出来的约束混进一张图而毫无提示。
//
// 做法:
//   --params <a.yaml>     读参数 (扁平的 key: value)
//   命令行其余选项仍然生效, 且**在 yaml 之后应用** = 覆盖。这样做实验不必每次新建文件。
//   每次运行把**生效后**的参数写到 <out>/params_effective.yaml 和 <store>/params.yaml。
//   写"生效后"而不是复制输入文件 —— 有命令行覆盖时, 只有生效值才是这次真正用的。
//   下次跑时若 <store>/params.yaml 已存在, 逐项比对并**列出差异**, 配准类参数变了会明确警告。
//
// 本项目已脱离 glim: 原来还有个 `config` 字段指向 glim 的配置目录(logging / sensors /
// global_mapping 那几个 json), 现已删除 —— 实测那些只被 glim 的 GlobalMapping/IMUIntegration
// 读, 本程序两个都不用。旧存档的 params.yaml 里若还带 config 键, 会被当作已废弃键忽略。
// -----------------------------------------------------------------------------
struct ParamField {
  char type;   // B=bool I=int D=double S=string, '#'=分段哨兵
  const char* name;
  void* ptr;
  const char* doc;   // 一行摘要, 写成 yaml 的行内注释; 详细说明见 --help
};

/// @brief struct Opt 的全部字段。**由脚本从结构体定义生成**, 124 个手抄必错。
///        加字段时同步加一行, 否则它不会出现在 yaml 里 (下面有自检: 数量对不上就报错)。
static std::vector<ParamField> paramFields(Opt& o) {
  const auto F = [](char t, const char* n, void* p, const char* d = "") {
    return ParamField{t, n, p, d};
  };
  return {
  F('S', "root", &o.root, "数据根目录"),
  F('S', "out", &o.out, "输出目录"),
  F('S', "data_mode", &o.data_mode, "clips | keyframe (默认 clips)"),
  F('S', "pose_dir", &o.pose_dir, "clips: egomotions | localized (默认 localized)"),
  F('S', "attitude_from", &o.attitude_from, "quat | euler (默认 euler)"),
  F('S', "lidar_dir", &o.lidar_dir, "clips: sensors/ 下哪个雷达 (默认 fuse_lidar)"),
  F('S', "pose_src", &o.pose_src, "keyframe: sparse | odoms (默认 sparse)"),
  F('I', "sa", &o.sa, "A 的 session 下标。**只有非增量的单点模式用**, --incr 下无效"),
  F('I', "sb", &o.sb, "B 的 session 下标。**只有非增量的单点模式用**, --incr 下无效"),
  F('D', "cx", &o.cx, "邻域中心 x (命令行 --center x,y)。**只有非增量的单点模式用**, --incr 下无效"),
  F('D', "cy", &o.cy, "邻域中心 y (命令行 --center x,y)。**只有非增量的单点模式用**, --incr 下无效"),
  F('D', "radius", &o.radius, "邻域半径, A 取 submap 用。**只有非增量的单点模式用**"),
  F('D', "b_radius", &o.b_radius, "B 取待配准帧的半径。**只有非增量的单点模式用**"),
    F('#', "按'B 帧位姿 vs 拼 submap 用到的 A 关键帧位姿'的关系筛", nullptr),
  F('D', "b_max_dist", &o.b_max_dist, "B 帧到**最近的 A 关键帧**的距离上限 (默认 5, <=0 关闭)"),
  F('D', "b_max_dyaw", &o.b_max_dyaw, "与最近 A 关键帧的航向差上限 (默认 180 = 关闭; 分布照样打印)"),
  F('I', "b_bidir", &o.b_bidir, "跨session 是否也收**反向**共位 (航向差 >= 180 - b_max_dyaw)。默认 0。"),
  F('I', "cross_per_sess", &o.cross_per_sess, "1 = 每个被并入帧对**每个**之前的 session 各建一条边 (默认 0 = 只对最近的那个)"),
    F('#', "跨 session 配准: 两端都先做局部 BA, 再 submap2submap", nullptr),
  F('I', "cross_ba", &o.cross_ba, "目标端 submap 先做一次**联合局部 BA** 再拼 (默认 0)。"),
  F('I', "cross_ba_edges", &o.cross_ba_edges, "目标端 BA 产出的**跨session** ref<->ref 相对位姿也发成约束 (默认 1)。"),
  F('I', "cross_ba_max", &o.cross_ba_max, "目标端做 BA 的帧数上限 (默认 80)。150 帧同时带协方差+体素图约 1.2GB。"),
  F('I', "cross_src_win", &o.cross_src_win, "源端也取当前帧 +-n 帧拼 submap (先局部 BA), 做 submap2submap (默认 0=单帧)。"),
  F('D', "frame_voxel", &o.frame_voxel, "单帧体素 (默认 0.25)"),
  F('D', "submap_voxel", &o.submap_voxel, "submap 体素 = target 点间距 (默认 0.15)"),
  F('D', "min_range", &o.min_range, "单帧点的最小距离 (m), <=0 = 关闭"),
  F('D', "max_range", &o.max_range, "单帧点的最大距离 (m), <=0 = 关闭"),
  F('D', "tgt_voxel", &o.tgt_voxel, "target 体素图分辨率 (默认 0.9)"),
  F('I', "iters", &o.iters, "VGICP 迭代 (默认 20)"),
  F('B', "fine", &o.fine, "1 = 做第二段 GICP 精化 (默认开; VGICP 的精度上限是体素尺度)。--no_fine 关"),
  F('D', "fine_corr", &o.fine_corr, "GICP 最大对应距离 (默认 0.5)"),
  F('I', "fine_iters", &o.fine_iters, "第二段 GICP 精化的迭代次数"),
  F('D', "min_inlier", &o.min_inlier, "采纳阈值 (默认 0.4)"),
  F('D', "max_corr", &o.max_corr, "水平修正上限 (默认 2.0)"),
  F('D', "z_above", &o.z_above, "量 nn 时只统计**高于地面** z_above 的点 (m) —— 路面点会把差异平均掉"),
  F('B', "clamp_planar", &o.clamp_planar, "1 = 平面压制: z/roll/pitch 取 INS 不动 (默认开)。--no_clamp 关"),
  F('B', "dyn_filter", &o.dyn_filter, "1 = 剔动态物体 (clips 用 annotations/3dod 的 3D 框, 默认开)。--no_dyn_filter 关"),
  F('D', "dyn_ts_tol", &o.dyn_ts_tol, "动态框与点云的时间戳容差 (默认 0.02)"),
  F('I', "dump_max", &o.dump_max, "最多存多少帧的前后点云+图 (默认 200)"),
  F('D', "max_io_fail", &o.max_io_fail, "帧读取失败率上限, 超了就中止且不写存档 (<=0 不检查)"),
  F('I', "frame_cache_mb", &o.frame_cache_mb, "帧缓存上限 (MB, 0=关)。同一帧在一次运行里被读 12~16 次"),
  F('I', "num_threads", &o.num_threads, "线程数 (默认 6)"),
  F('B', "scan_overlap", &o.scan_overlap, "只扫两条轨迹的共位情况然后退出, 不做配准。"),
  F('D', "scan_step", &o.scan_step, "--scan 沿轨迹的采样步长 (m)"),
    F('#', "A 的 submap 内部 BA (Ceres: INS 先验 + 帧间 GICP + 视觉重投影)", nullptr),
  F('B', "ba", &o.ba, "对 A 的邻域帧做**submap 内部 BA** 再拼 submap (默认关)"),
  F('D', "ba_min_overlap", &o.ba_min_overlap, "帧间建约束的最小重叠率 (默认 0.2)"),
  F('D', "ba_pair_voxel", &o.ba_pair_voxel, "BA 内帧间 GICP 的体素 (m)"),
  F('I', "ba_pair_iters", &o.ba_pair_iters, "帧间/回环 VGICP 的 LM 迭代次数 (默认 20)"),
  F('D', "ba_max_corr", &o.ba_max_corr, "BA 内建帧间约束的最大帧间距离 (m), 超过就不建这条边"),
    F('#', "以下默认值由**白化残差实测**校准而来, 不是拍的", nullptr),
  F('D', "ba_ins_xy", &o.ba_ins_xy, "INS 位置软锚 sigma (默认 0.40, 实测校准值)"),
  F('D', "ba_rel_t", &o.ba_rel_t, "帧间相对位姿平移 sigma (默认 0.035, 实测校准值)"),
  F('D', "ba_rel_r", &o.ba_rel_r, "帧间旋转 sigma (默认 0.002, 实测校准值)"),
  F('D', "ba_att_w", &o.ba_att_w, "姿态先验权重 (默认 275, 实测校准值; 原 50 偏轻 5.5 倍)"),
  F('D', "ba_huber", &o.ba_huber, "Ceres 的 Huber 阈值 (0 = 不用鲁棒核)"),
  F('D', "ba_chi2", &o.ba_chi2, ">0 时按卡方阈值剔除异常帧间约束后再解一轮 (默认 0)"),
  F('I', "ba_iters", &o.ba_iters, "Ceres 迭代上限"),
  F('B', "ba_visual", &o.ba_visual, "BA 里再加视觉重投影约束 (隐含 --ba)"),
  F('D', "ba_sigma_px", &o.ba_sigma_px, "视觉重投影的 sigma, 单位像素 (默认 3.6, 实测校准值)"),
  F('S', "vis_cams", &o.vis_cams, "视觉用哪些相机 (默认全部)"),
  F('D', "yaw_fix_deg", &o.yaw_fix_deg, "给雷达外参补 yaw 修正, 右乘到 T_w_l (默认 0)"),
  F('I', "opt_ext", &o.opt_ext, "把**雷达外参 T_v_l 也放进 BA** 一起优化 (0=不动 1=只旋转 2=旋转+平移)"),
    F('#', "导出点云上色", nullptr),
  F('B', "color", &o.color, "1 = 导出的点云上色 (默认开)。--no_color 关"),
  F('S', "color_cams", &o.color_cams, "上色用哪些相机 (默认全部)"),
  F('D', "color_z_min", &o.color_z_min, "上色时相机坐标系下的最小深度 (m)"),
  F('D', "color_max_range", &o.color_max_range, "超过这个距离不上色的硬阈值 (默认 20)。地面实际由下面的入射角门决定。"),
  F('D', "color_smear_max", &o.color_smear_max, "**入射角门** (默认 0.10): 1 像素的图像误差在该点表面上造成的位移"),
  F('D', "loop_min_arc", &o.loop_min_arc, "回环两帧之间的**弧长**下限(m)。默认 0 = 只用'序号差 > intra_win' 判基线。"),
  F('I', "export_raw", &o.export_raw, ".las 和 _rgb.pcd 用**原始点**写 (默认 1): 只过 min/max_range 和"),
  F('I', "export_before", &o.export_before, "是否导出 _before 那一份 (默认 0 = 不导)。"),
  F('I', "do_export", &o.do_export, "0 = **完全不导出点云** (命令行叫 --export)。参数扫描/sigma 校准时用"),
  F('I', "las", &o.las, "除 .pcd 外再写一份 .las (LAS 1.2 Point Format 2): **一个文件**同时"),
  F('I', "utm_zone", &o.utm_zone, "UTM 带号, 只用来往 las 里写 CRS 声明 (EPSG = 32600+n, WGS84 北半球)。"),
  F('D', "color_ground_h", &o.color_ground_h, "车体系高度低于此值的点算'地面点', 才走入射角门 (默认 0.5)"),
  F('D', "color_occ_tol", &o.color_occ_tol, "遮挡判定的深度容差 (m): 比 z-buffer 深这么多就算被挡住"),
  F('I', "color_occ_px", &o.color_occ_px, "遮挡 z-buffer 的像素降采样倍数"),
  F('I', "color_drop", &o.color_drop, "1 = 丢弃没上到色的点; 0 = 保留"),
    F('#', "原始点导出 (不做体素滤波, 只裁到一个方框内)", nullptr),
  F('D', "raw_x", &o.raw_x, "--raw_export 方框中心 x (m)"),
  F('D', "raw_y", &o.raw_y, "--raw_export 方框中心 y (m)"),
  F('D', "raw_size", &o.raw_size, "--raw_export 方框边长 (m); <=0 = 不额外导出这一块"),
  F('I', "calib_rounds", &o.calib_rounds, "按白化残差 rms 自校准各类约束 sigma 的轮数 (默认 0 = 不校准)"),
  F('I', "calib_priors", &o.calib_priors, "1=连 INS/姿态先验也校准 (默认 0; 实测会一路收紧到把位姿钉死)"),
    F('#', "增量建图 (多 session 依次并入)", nullptr),
  F('B', "incr", &o.incr, "开启增量模式 (取代 --joint 的两 session 逻辑)"),
  F('S', "pose_store", &o.pose_store, "位姿存档目录 (默认 <out>/poses)。**有存档 = 已优化, 本次跳过**"),
  F('S', "redo", &o.redo, "强制重做这些 session (即使有存档)"),
  F('S', "sess_order", &o.sess_order, "session 顺序 (逗号分隔的目录名); 默认按目录序"),
  F('B', "list_done", &o.list_done, "只打印'哪些已优化/哪些待优化'然后退出, 不干活"),
  F('S', "load_g2o", &o.load_g2o, "载入已有位姿图(需同名 _index.csv); 里面的 session 视为已优化"),
  F('I', "freeze_prev", &o.freeze_prev, "1 = 老 session 的位姿在联合优化里钉死 (默认 0 = 一起优化)"),
    F('#', "同 session 回环 (独立于分窗逻辑的一路)", nullptr),
  F('B', "loop", &o.loop, "1 = 做同 session 回环 (默认开)。--no_loop 关"),
  F('D', "loop_dist", &o.loop_dist, "空间距离上限 (默认 12)"),
  F('I', "loop_min_gap", &o.loop_min_gap, "序号差下限 (默认 0 = 用 intra_win; 低于它的由分窗逻辑覆盖)"),
  F('D', "loop_step", &o.loop_step, "沿弧长每隔多少米开一次回环尝试 (默认 20)"),
  F('I', "loop_per_frame", &o.loop_per_frame, "每次尝试最多留几条 (默认 2)"),
  F('I', "loop_max", &o.loop_max, "候选总数上限 (默认 6000)"),
  F('D', "loop_min_inlier", &o.loop_min_inlier, "回环的 inlier 门 (默认 0.6, 比窗口内的 0.2 重叠门严得多)"),
  F('D', "loop_min_inlier_rev", &o.loop_min_inlier_rev, "**反向**回环的 inlier 门 (比同向松: 反向照到的是相反那一面)"),
  F('I', "loop_submap", &o.loop_submap, "回环用 **submap2submap**: 每端取 +-n 帧拼局部 submap 再配 (0=单帧, 默认)"),
  F('D', "loop_submap_radius", &o.loop_submap_radius, "拼 submap 时邻帧到中心帧的距离上限 (默认 25)。"),
  F('I', "loop_ba", &o.loop_ba, "拼 loop submap 之前先对那 (2n+1) 帧做一次**局部 BA** (默认 1)。"),
  F('D', "loop_sigma_t", &o.loop_sigma_t, "回环约束的平移 sigma。<=0 = 沿用 --ba_rel_t (默认)。"),
  F('D', "loop_sigma_r", &o.loop_sigma_r, "回环约束的旋转 sigma。<=0 = 沿用 --ba_rel_r"),
  F('D', "loop_anti_w", &o.loop_anti_w, "反向回环的**权重**相对同向的倍数 (默认 0.5 = 一半)。"),
  F('I', "dump_loop", &o.dump_loop, "**debug**: 落盘前 n 个回环候选 (0=关)。注意 --dump_cross 只管"),
  F('I', "dump_pcd", &o.dump_pcd, "dump 时是否**也存 pcd** (默认 0 = 只存 png)。"),
    F('#', "约束去重 / 均匀化 (四维 NMS)", nullptr),
  F('D', "nms_loop", &o.nms_loop, "回环用**四维 NMS** 去重 (m); <=0 = 用旧的中点聚类 loop_cluster_*"),
  F('D', "nms_cross", &o.nms_cross, "跨session 用四维 NMS 去重 (m); <=0 = 不去重 (旧行为, 每帧一条)"),
  F('S', "nms_win_deltas", &o.nms_win_deltas, "同session 窗口内**只建**这些帧号差的约束; 空 = 全建(默认)。"),
  F('D', "loop_cluster_dist", &o.loop_cluster_dist, "回环候选按位置聚类的半径 (m)"),
  F('I', "loop_cluster_max", &o.loop_cluster_max, "每个聚类里最多留几条回环候选"),
  F('I', "dump_cross", &o.dump_cross, "**debug**: 每次 scan-to-submap 都存下来, 最多 n 帧 (默认 0=关)"),
  F('D', "drop_isolated", &o.drop_isolated, "连通半径 (默认 10, 0=关)。相距 <=m 的帧算相连, **只保留最大的"),
  F('D', "drop_static", &o.drop_static, "连通域自身空间尺度小于这个就整块丢 (默认 5)。"),
  F('I', "drop_frames", &o.drop_frames, "**默认 0 = 一帧都不丢**; 连通域/静止块只分析并打印, 不真丢"),
    F('#', "旧路径: 两 session 一次性联合 —— 已被 --incr 取代, 只留作对照实验", nullptr),
  F('B', "joint", &o.joint, "选**旧路径** runJoint (两 session 一次性联合)。与 --incr 平行, 二者只能选一个"),
    F('#', "加载期帧筛选 (--joint / --incr 共用)", nullptr),
  F('D', "load_roi_x", &o.load_roi_x, "--load_roi 方框中心 x (m)"),
  F('D', "load_roi_y", &o.load_roi_y, "--load_roi 方框中心 y (m)"),
  F('D', "load_roi_size", &o.load_roi_size, "--load_roi 方框边长 (m); <=0 = 用全部帧。**改它等于换了帧集**"),
    F('#', "同 session 帧间约束 (--incr 的主力: buildIntra / buildLoop / localBA 都用)", nullptr),
  F('D', "intra_pair_dist", &o.intra_pair_dist, "同 session 内建 GICP 约束的最大帧间距离 (默认 12)"),
  F('I', "intra_win", &o.intra_win, "分窗处理的窗口帧数, 窗口间 50% 重叠 (默认 40)"),
    F('#', "跨 session 约束: 重叠区域的划分 (--incr 的 buildCross 用)", nullptr),
  F('D', "region_step", &o.region_step, "沿共位段每隔多少米开一个重叠区域 (默认 150)"),
  F('D', "region_radius", &o.region_radius, "每个重叠区域的半径 (默认 40)"),
  F('I', "region_max_frames", &o.region_max_frames, "每个区域拼 submap 最多用多少参照帧 (默认 150, 0=不限)"),
    F('#', "跨 session 约束的权重 (--incr 的最终联合优化用)", nullptr),
  F('D', "cross_sigma_xy", &o.cross_sigma_xy, "跨 session 约束的 sigma (默认 0.09)"),
  F('D', "cross_sigma_r", &o.cross_sigma_r, "跨 session 约束的旋转 sigma (默认 0.005)"),
  };
}

/// @brief 把生效后的参数写成扁平 yaml。
static bool saveParams(const fs::path& f, Opt& o, const char* note) {
  std::ofstream os(f.string());
  if (!os) {
    printf("  !! 参数写不出去: %s\n", f.string().c_str());
    return false;
  }
  os << "# " << note << "\n";
  os << "# 这是**生效后**的参数 (yaml + 命令行覆盖之后的实际取值), 不是输入文件的副本。\n";
  os << "# 下次增量时用 --params 指向它, 就能保证前后参数完全一致。\n";
  os << "# 每一项的含义和\"为什么是这个值\"见 local_align --help。\n";
  os.precision(10);
  for (const auto& p : paramFields(o)) {
    if (p.type == '#') {                     // 分段哨兵: 只为可读性, 124 行扁平文件没法看
      os << "\n# ---- " << p.name << " ----\n";
      continue;
    }
    std::ostringstream kv;
    kv.precision(10);
    kv << p.name << ": ";
    switch (p.type) {
      case 'B': kv << (*static_cast<bool*>(p.ptr) ? "true" : "false"); break;
      case 'I': kv << *static_cast<int*>(p.ptr); break;
      case 'D': kv << *static_cast<double*>(p.ptr); break;
      default: {
        const std::string& v = *static_cast<std::string*>(p.ptr);
        kv << '"' << v << '"';
      }
    }
    // 行内注释。摘要和 --help 是同一句话; 详细说明(为什么是这个值、反例)仍只在 --help 里。
    os << kv.str();
    if (p.doc && *p.doc) {
      for (int pad = static_cast<int>(kv.str().size()); pad < 30; pad++) os << ' ';
      os << "  # " << p.doc;
    }
    os << "\n";
  }
  return os.good();
}

/// @brief 读扁平 yaml 进 Opt。**未知键直接失败** —— 与命令行"未知参数即报错"同一个理由:
///        拼错一个键而程序照跑, 得到的是"改了参数却没生效"的假结论。
static bool loadParams(const fs::path& f, Opt& o) {
  YAML::Node n;
  try {
    n = YAML::LoadFile(f.string());
  } catch (const std::exception& e) {
    printf("  !! 参数文件读不了 %s : %s\n", f.string().c_str(), e.what());
    return false;
  }
  if (!n.IsMap()) {
    printf("  !! 参数文件不是一个 map: %s\n", f.string().c_str());
    return false;
  }
  const auto fl = paramFields(o);
  std::map<std::string, const ParamField*> by;
  for (const auto& p : fl) {
    if (p.type != '#') by[p.name] = &p;
  }
  std::size_t hit = 0;
  bool bad = false;
  for (const auto& kv : n) {
    const std::string k = kv.first.as<std::string>();
    // 已删掉的参数: 旧存档的 params.yaml 里还带着它们。未知键是硬失败(防拼错), 但**已废弃**
    // 的键不该让旧存档整个读不进来 —— 那正好砸在"下次增量直接用上次的 params.yaml"上。
    // 只放确实删过的键, 不是"允许任意未知键"的后门。
    static const std::set<std::string> kRetired = {
      "render_sample_pairs",   // 曾是"生成样例俯视图", 实现早已删掉, 参数空留
      "config",                // glim 的配置目录。本项目已脱离 glim, 不再需要
      // 回环不再判航向 (见 buildLoop): 同向/反向两个门中间的死区正好吃掉十字路口的
      // 垂直经过, 而实测那些边配准质量与反向边相当。现在只按距离+里程收, 统一用
      // loop_min_inlier_rev 的门和 loop_anti_w 的权重。
      "loop_dyaw", "loop_bidir", "loop_any_dyaw",
    };
    if (kRetired.count(k)) {
      printf("  参数 %s 已废弃(功能已删), 忽略\n", k.c_str());
      continue;
    }
    const auto it = by.find(k);
    if (it == by.end()) {
      printf("  !! 参数文件里有未知键: %s\n", k.c_str());
      bad = true;
      continue;
    }
    try {
      switch (it->second->type) {
        case 'B': *static_cast<bool*>(it->second->ptr) = kv.second.as<bool>(); break;
        case 'I': *static_cast<int*>(it->second->ptr) = kv.second.as<int>(); break;
        case 'D': *static_cast<double*>(it->second->ptr) = kv.second.as<double>(); break;
        default: *static_cast<std::string*>(it->second->ptr) = kv.second.as<std::string>();
      }
    } catch (const std::exception&) {
      printf("  !! 键 %s 的值解析失败\n", k.c_str());
      bad = true;
      continue;
    }
    hit++;
  }
  if (bad) return false;
  std::size_t nf = 0;
  for (const auto& p : fl) nf += (p.type != '#');
  printf("  参数: %s (%zu/%zu 项; 未列出的用默认值)\n", f.string().c_str(), hit, nf);
  return true;
}

/// @brief 和存档里的参数逐项比对, 把差异列出来。
///
/// 这是**增量建图最容易踩的坑**的唯一防线: 存档复用只看帧集是否 100% 命中, 所以改配准
/// 参数不会让存档失效 —— 旧参数配出来的约束会和新参数配出来的混进同一张图, 而没有任何提示。
/// 这里把差异分成两类: 影响**已存档约束**的(配准/滤波类) 和 只影响本次求解的(sigma/导出类)。
static bool diffParams(const fs::path& f, Opt& o) {
  if (!fs::exists(f)) return false;
  Opt old;                       // 从存档参数复原出一份
  if (!loadParams(f, old)) {
    printf("  !! 存档里的参数读不出来, 跳过比对: %s\n", f.string().c_str());
    return false;
  }
  // 影响已存档约束的键: 改了它们, 存档里的测量就不是同一套流程产出的
  static const std::set<std::string> kReg = {
    "root", "data_mode", "pose_dir", "attitude_from", "lidar_dir", "pose_src",
    "frame_voxel", "submap_voxel", "tgt_voxel", "min_range", "max_range", "dyn_filter",
    "dyn_ts_tol", "iters", "fine", "fine_corr", "fine_iters", "min_inlier", "max_corr",
    "clamp_planar", "z_above", "intra_win", "intra_pair_dist", "ba_min_overlap",
    "ba_pair_voxel", "ba_pair_iters", "ba_max_corr", "nms_win_deltas", "nms_loop", "nms_cross",
    "loop", "loop_dist", "loop_min_gap", "loop_step", "loop_per_frame", "loop_max",
    "loop_min_inlier", "loop_min_inlier_rev", "loop_min_arc", "loop_submap",
    "loop_submap_radius", "loop_ba", "loop_cluster_dist", "loop_cluster_max",
    "b_max_dist", "b_max_dyaw", "b_bidir", "cross_per_sess", "cross_ba", "cross_ba_edges",
    "cross_ba_max", "cross_src_win", "region_step", "region_radius", "region_max_frames",
    "drop_isolated", "drop_static", "drop_frames", "load_roi_x", "load_roi_y", "load_roi_size",
    "yaw_fix_deg", "opt_ext", "ba_visual", "ba_sigma_px",
  };
  const auto fn = paramFields(o);
  const auto fo = paramFields(old);
  std::vector<std::string> reg, oth;
  for (std::size_t k = 0; k < fn.size(); k++) {
    if (fn[k].type == '#') continue;
    std::string a, b;
    switch (fn[k].type) {
      case 'B': a = *static_cast<bool*>(fn[k].ptr) ? "true" : "false";
                b = *static_cast<bool*>(fo[k].ptr) ? "true" : "false"; break;
      case 'I': a = std::to_string(*static_cast<int*>(fn[k].ptr));
                b = std::to_string(*static_cast<int*>(fo[k].ptr)); break;
      case 'D': { char t1[48], t2[48];
                  std::snprintf(t1, sizeof(t1), "%.10g", *static_cast<double*>(fn[k].ptr));
                  std::snprintf(t2, sizeof(t2), "%.10g", *static_cast<double*>(fo[k].ptr));
                  a = t1; b = t2; break; }
      default: a = *static_cast<std::string*>(fn[k].ptr);
               b = *static_cast<std::string*>(fo[k].ptr);
    }
    if (a == b) continue;
    // 路径类不算差异: 换输出目录、把存档搬个位置都是正常操作, 报出来只是噪音。
    // (`params` / `save_params` 本身**不在 registry 里** —— 否则存档的 yaml 会带着
    //  save_params 字段, 下次 --params 读进来就直接写文件退出了。)
    static const std::set<std::string> kSkip = {"out", "pose_store"};
    if (kSkip.count(fn[k].name)) continue;
    char line[256];
    std::snprintf(line, sizeof(line), "%-22s 存档 %-16s -> 本次 %s", fn[k].name, b.c_str(), a.c_str());
    (kReg.count(fn[k].name) ? reg : oth).push_back(line);
  }
  if (reg.empty() && oth.empty()) {
    printf("  参数与存档**完全一致**\n");
    return false;
  }
  if (!reg.empty()) {
    printf("\n  !! 与存档相比, **影响已存档约束**的参数变了 %zu 项:\n", reg.size());
    for (const auto& l : reg) printf("       %s\n", l.c_str());
    printf("     存档里的约束是按**旧参数**配出来的, 而复用判据只看帧集是否 100%% 命中 ——\n"
           "     所以它不会自动失效。混用两套参数配出来的约束, 结果不可解释。\n"
           "     要么 --redo 把相关 session 重做, 要么换一个空的 --pose_store。\n");
  }
  if (!oth.empty()) {
    printf("  与存档相比, 只影响本次求解/导出的参数变了 %zu 项 (安全):\n", oth.size());
    for (const auto& l : oth) printf("       %s\n", l.c_str());
  }
  return !reg.empty();
}

/// @brief 打印命令行帮助。默认值和"为什么是这个值"都写在里面 —— 这个项目里绝大多数
///        参数是实测校准出来的, 帮助文本就是那些实测结论的落点。
static void usage() {
  std::cout << R"(用法: local_align --center x,y [选项]

  在指定位置的邻域内: A 的帧拼成 submap, B 的帧逐一配准到它。
  只做这两步, 不做联合优化 —— 变量少, 每一步都能单独看。

  --params <a.yaml>     **从 yaml 读全部参数** (扁平 key: value; 键名 = 下面各选项去掉 --)。
                        123 个参数靠命令行传等于每次重打一长串, 而增量建图的要害是**前后
                        两次参数必须完全一致** —— 存档复用只看"帧集是否 100%% 命中",
                        改配准参数**不会**让存档失效, 很容易把两套参数配出来的约束混进一张图。
                        命令行其余选项**在 yaml 之后应用 = 覆盖**, 做实验不必每次新建文件。
                        每次运行把**生效后**的参数写到 <out>/params_effective.yaml 和
                        <pose_store>/params.yaml; 存档里已有则逐项比对并列出差异,
                        配准类参数变了会明确警告。
                        未知键直接失败 —— 拼错一个键而照跑, 得到的是"改了却没生效"的假结论。
  --save_params <a.yaml>  把当前(默认值+本次命令行)参数写成 yaml 然后退出。用它生成初版模板。
  --root <dir>          数据根目录
  --out <dir>           输出目录
  --center <x,y>        邻域中心 (必填; 坐标系与主程序日志里的"轨迹包围盒"一致)
  --radius <m>          邻域半径, A 取 submap 用 (默认 30)
  --b_radius <m>        B 取待配准帧的半径 (默认同 radius)
  --b_max_dist <m>      B 帧到**最近的 A 关键帧**的距离上限 (默认 5, <=0 关闭)
                        只按中心半径选 B 帧不够: 离 A 的帧太远的 B 帧看的是 A 没覆盖的地方,
                        实测 85% 源点会被覆盖门剔掉, 剩下的样本又少又偏。
  --b_max_dyaw <deg>    与最近 A 关键帧的航向差上限 (默认 180 = 关闭; 分布照样打印)
                        反向行驶时同一根杆子被照射的是相反那一面、采样密度分布对调、
                        遮挡相反 —— 残差里有配准消除不掉的系统性成分。
  --sa <n> --sb <n>     A / B 的 session 下标 (默认 0 / 1)

  --data_mode <s>       clips | keyframe (默认 clips)
  --pose_dir <s>        clips: egomotions | localized (默认 localized)
  --attitude_from <s>   quat | euler (默认 euler)
                        !! localized 必须配 euler: 实测它的 orientation 四元数与同文件的
                           euler_angles 自相差 4.44 度中位 / 25.5 度最大, 坏的是四元数。
  --lidar_dir <s>       clips: sensors/ 下哪个雷达 (默认 fuse_lidar)
  --pose_src <s>        keyframe: sparse | odoms (默认 sparse)

  --frame_voxel <m>     单帧体素 (默认 0.25)
  --submap_voxel <m>    submap 体素 = target 点间距 (默认 0.15)
  --tgt_voxel <m>       target 体素图分辨率 (默认 0.9)
                        !! 必须 >= 3 x max(frame_voxel, submap_voxel), 否则 inlier 静默塌掉
  --iters <n>           VGICP 迭代 (默认 20)
  --no_fine             关掉第二段 GICP 精化 (默认开; VGICP 的精度上限是体素尺度)
  --fine_corr <m>       GICP 最大对应距离 (默认 0.5)
  --min_inlier <r>      采纳阈值 (默认 0.4)
  --max_corr <m>        水平修正上限 (默认 2.0)
  --no_clamp            不做平面压制, 采纳完整 6-DOF (默认压制: z/roll/pitch 取 INS)
  --no_dyn_filter       不剔动态物体 (默认剔; clips 用 annotations/3dod 的 3D 框)
  --dump_max <n>        最多存多少帧的前后点云+图 (默认 200)
  --ba                  对 A 的邻域帧做**submap 内部 BA** 再拼 submap (默认关)
                        Ceres: INS 位置软锚(z x0.75) + 重力分解姿态先验(yaw 自由)
                              + 帧间 VGICP 相对位姿 + Huber (+ 可选卡方剔除)
                        为什么需要: 实测共位段上 submap 自身重影 0.176m, 而 B 帧配准前
                        离它只有 0.120m —— 配准精度的上限是**参照物的清晰度**。
  --ba_visual           BA 里再加视觉重投影约束 (隐含 --ba)
                        补的是沿路方向 —— 那是激光的退化方向(沿路平移最近邻代价近乎平坦)。
  --ba_sigma_px <px>    视觉 sigma (默认 3.6, 实测校准值)。日志会打白化 rms:
                        rms 偏离 1 就照那个比例调, 别按"想让它多重要"给。
  --ba_ins_xy <m>       INS 位置软锚 sigma (默认 0.40, 实测校准值)
  --ba_rel_t <m>        帧间相对位姿平移 sigma (默认 0.035, 实测校准值)
                        这个 3.5cm 就是**关键帧↔关键帧的配准可重复性**;
                        之前项目里沿用的 0.20m 是 submap↔submap 的数, 大了 6 倍。
  --ba_rel_r <rad>      帧间旋转 sigma (默认 0.002, 实测校准值)
  --ba_att_w <r>        姿态先验权重 (默认 275, 实测校准值; 原 50 偏轻 5.5 倍)
  --ba_min_overlap <r>  帧间建约束的最小重叠率 (默认 0.2)
  --ba_pair_iters <n>   帧间/回环 VGICP 的 LM 迭代次数 (默认 20)
  --dyn_ts_tol <s>      动态框与点云的时间戳容差 (默认 0.02)
  --ba_chi2 <r>         >0 时按卡方阈值剔除异常帧间约束后再解一轮 (默认 0)
  --vis_cams <a,b>      视觉用哪些相机 (默认全部)
  --color               给导出的点云上色 (相机内外参投影, 另存 *_rgb.pcd)
  --no_color            关掉上色 (默认是开的; 只想看几何/跑对照实验时用)
  --color_cams <a,b>    上色用哪些相机 (默认全部)
  --color_max_range <m> 超过这个距离不上色的硬阈值 (默认 20)。地面实际由下面的入射角门决定。
  --color_smear_max <m> **入射角门** (默认 0.10): 1 像素的图像误差在该点表面上造成的位移
                        超过此值就不上色。
                        [实测] 相机高 1.805m、fy=774.5。地面点在相机里是掠射的, 位移是
                        d^2/(f*h):  10m -> 0.072   20m -> 0.287   30m -> 0.646   40m -> 1.148 m
                        也就是 30m 处**仅像素量化**就把漆面颜色抹开 0.65m, 标定再准也没用 ——
                        这正是路口斑马线在彩色点云里发虚的主因。同一块地面用雷达强度渲出来,
                        车道线/停止线/斑马线边缘都很锐利, 说明几何是对齐的, 糊在上色这一步。
                        为什么不用一刀切的距离: 同样 30m, 竖直面 1 像素只对应 d/f=0.039m
                        完全可用, 地面却是 0.646m 完全不可用 —— 判据是入射角, 不是距离。
                        按 0.10m 反解: 地面约在 12m 截断, 竖直面不受限。
  --color_ground_h <m>  车体系高度低于此值的点算"地面点", 才走入射角门 (默认 0.5)
  --cross_ba <0|1>     目标端 submap 先做一次**联合局部 BA** 再拼 (默认 0)。
                       为什么: 混合 submap 把 A、B 在该位置的不一致烙进 target, 而配准精度
                       上限就是 target 的清晰度。[实测] 三 session 覆盖处两两 nn 0.064~0.139,
                       而跨session 配准后的 nn 就是 0.07~0.10 —— 已经贴在上限上。
                       **注意**: 开它会连带发出跨session ref<->ref 约束 (见 --cross_ba_edges),
                       那不是可选项 —— BA 把 A、B 相对挪了, 不把这个挪动告诉图, C 的边就和
                       图里的 ref 位姿不在同一个规范系里。
  --cross_ba_edges <0|1>  目标端 BA 产出的**跨session** ref<->ref 相对位姿也发成约束 (默认 1)。
                       关掉它 = 明知规范系不一致还硬用, 只在做对照实验时才关。
  --cross_ba_max <n>   目标端做 BA 的帧数上限 (默认 80)。150 帧同时带协方差+体素图约 1.2GB。
  --cross_src_win <n>  源端也取当前帧 +-n 帧拼 submap (先局部 BA), 做 submap2submap (默认 0=单帧)。
                       [参照] 回环那边同样的改动让 inlier 0.585->0.790、nn 0.204->0.119。
  --b_bidir <0|1>      跨session 是否也收**反向**共位 (航向差 >= 180 - b_max_dyaw)。默认 0。
                       比回环那边安全: 航向门只决定"算不算共位/边挂哪个参照帧",
                       **不影响配准** —— submap 只按距离筛帧, 本来就含两个方向的点。
                       [实测] 路口处 jjst3/jjst5 走一向、jjst4 走反向, 于是 jjst4 与它们
                       在那个路口一条约束都没有, 只能绕 jjst2 —— 斑马线不一致就是这么来的。
  --cross_per_sess <0|1>  1 = 每个被并入帧对**每个**之前的 session 各建一条边 (默认 0 =
                       只挂全局最近的那一个)。
                       [实测] jjst5 有 799 帧同时被 2 个之前 session 覆盖, 只建了 1 条 ->
                       漏 924 条 (比实建多 48%)。submap 用了那些点, 图里却没有边。
                       同一次配准派生的 N 条边共享同一个配准噪声, 所以它们的 sigma 自动
                       乘 sqrt(N), 免得把一次测量的权重算 N 遍。
  --nms_loop <m>       回环用**四维 NMS** 去重 (m); <=0 = 用旧的中点聚类 loop_cluster_*
                       抑制条件: 已有一条边时, 新边的**两端都**在 R 内才算重复。
                       [实测] 相邻帧间距 p75=4.6m p90=5.8m (11.8m/s x 0.5s), 所以 R 要 >=6
                       才压得掉"±1 帧"那种重复; R=4 只去掉 19/901 条, R=6 去掉一半。
  --nms_cross <m>      跨session 用四维 NMS 去重 (m); <=0 = 不去重 (旧行为, 每帧一条)
                       **要比回环的小**: 跨session 边两端由 b_max_dist(10m) 约束着本来就
                       挨着, 四维条件退化成"每 R 米留一条"。R=2 就能把病态堆(红灯前停着的
                       几十帧全建边, 实测单格最多 129 条)削平, 而正常位置的中位数不变。
  --nms_win_deltas <a,b,c>  同session 窗口内**只建**这些帧号差的约束; 空 = 全建(默认)。
                       在**候选阶段**生效, 所以省的是配准时间(成本大头)。
                       [实测] 同session边里 Δ=1 只占 8.2%、Δ>=2 占 92%, 给 "1" 能省 92%
                       的帧间配准。但 Δ=1 的链 sigma=0.035 串 40 帧累计 0.22m, 而一条
                       Δ=40 的直连边只有 0.035 —— 中等 Δ 抑制的是链上累计漂移, 不是纯冗余。
                       建议 "1,5,20,40" 这种对数梯度而不是光给 "1"。
  --loop_submap <n>    回环用 **submap2submap**: 每端取 +-n 帧拼局部 submap 再配 (0=单帧, 默认)
                       为什么可能更好: 反向重访时单帧的共同可见表面很少 —— 同一根杆子被照射的
                       是相反那一面, 近密远疏的采样分布对调, 遮挡也相反。拼成一段路之后
                       共同表面大得多, 且格内多次采样取均值本身就降噪。
                       代价: 每个候选要读 2*(2n+1) 帧而不是 2 帧, IO 涨 (2n+1) 倍。
                       **上限是 submap 自身的清晰度** —— submap 用的是 INS 相对位姿(此时
                       还没解), 所以日志会打它的奇偶自测重影, 那个数就是这条约束的精度上限。
  --loop_submap_radius <m>  拼 submap 时邻帧到中心帧的距离上限 (默认 25)。
                       没有它的话, 在红灯前停着的那一段会把十几帧堆在同一个位置, 只增 IO 不增信息。
  --loop_ba <0|1>      拼 loop submap 之前先对那 (2n+1) 帧做一次**局部 BA** (默认 1)。
                       与窗口 BA 同一套逻辑(INS 先验 + 帧间 GICP + Ceres), 只是范围小。
                       [实测] 不做的话目标端 submap 自身重影中位 0.143 m —— 那就是回环
                       约束的精度上限, 比帧间可重复性 0.035 差 4 倍。
  --loop_sigma_t <m>   回环约束的平移 sigma。<=0 = 沿用 --ba_rel_t (默认)。
                       **应当按白化 rms 单独校准**: 日志现在把"同向回环"和"反向回环"
                       各自的 rms 分开打, 照它乘上去即可。
  --loop_sigma_r <rad> 回环约束的旋转 sigma。<=0 = 沿用 --ba_rel_r
  --loop_anti_w <r>    反向回环的**权重**相对同向的倍数 (默认 0.5 = 一半)。
                       权重 = 1/sigma^2, 所以 r=0.5 <=> sigma 乘 sqrt(2)=1.414。
  --dump_pcd <0|1>     dump 时是否**也存 pcd** (默认 0 = 只存 png)。
                       png 里已经有配准前/后的叠加(上下=前/后, 左右=全部点/地面以上),
                       判读够用; pcd 是体积大头 (全量下好几 GB)。
  --dump_loop <n>      **debug**: 落盘前 n 个回环候选 (0=关)。注意 --dump_cross 只管
                       跨 session 的 scan2submap, 与回环无关。输出到 <out>/loop/ :
                         l<序>_<same|cross|anti>_nn<配准后>_g<改善>_<OK|REJ>_i<>_j<>.png
                         同名 _A.pcd (目标端) / _B_before.pcd / _B_after.pcd
                       文件名把分数放最前, ls 天然按质量排序, ls -r 从最差看起。
  回环**不判航向**: 只按距离(--loop_dist)和里程(--loop_min_arc)收候选, 一律用
                        --loop_min_inlier_rev 的门 + --loop_anti_w 的权重。
                        为什么去掉航向门: 原来同向门(<=30度)与反向门(>=150度)之间留出死区,
                        十字路口垂直经过恰好落在里面 —— 实测 jjst2 在 (14,-450) 那个路口,
                        两次经过距离最近的一对相距仅 2.08 m、航向差 127 度, 一直没建约束。
                        关键帧有 INS 初值, 配准有好起点, 航向门只是多余的保险:
                        实测死区里 31 对帧对 100% 采纳, inlier 中位 0.713 (反向边 0.791)。
                        为什么统一用保守档: 有里程门之后"同向重访"几乎不出现(实测 jjst2 的
                        320 条回环边里同向 0 条 —— 同向又挨着的全是红灯停车的零基线对)。
  --loop_min_arc <m>   回环两帧之间的**弧长**下限(m)。默认 0 = 只用"序号差 > intra_win" 判基线。
                       [实测] 等灯 45 秒 = 90 多帧, 轻松满足序号差门而车根本没动 ——
                       序号差挡不住原地停车, 弧长才挡得住。50 左右是合适的值。
  --export <0|1>       0 = **完全不导出点云** (默认 1)。参数扫描/sigma 校准时用 ——
                       一次全量导出约 25GB/几十分钟, 而那些实验只看日志里的 rms 和位姿。
  --export_before <0|1>  是否导出 _before 那一份 (默认 0 = 不导)。
                       _before 用 INS 初值位姿, 只是对照; 导它要把全部帧再读一遍、再上一遍
                       色、再写一份 las/rgb —— 时间和磁盘都翻倍。要做优化前后对比才打开。
  --export_raw <0|1>   .las 和 _rgb.pcd 用**原始点**写 (默认 1): 只过 min/max_range 和
                       动态物体剔除, **不做体素滤波** —— 强度和点的分布都是原始测量值。
                       纯 xyz 的 .pcd 不受影响, 始终是体素后的(它是量重影/厚度的基准,
                       换掉就和之前所有测量不可比)。
                       [实测 jjst2] range[5,40] 把每帧 20.4 万点砍到 6.75 万, 全量 1.69 亿点:
                       las 4.4 GB/份, before+after 8.8 GB。内存不涨 —— 走流式写盘。
  --las <0|1>          除 .pcd 外再写一份 .las (LAS 1.2 Point Format 2): **一个文件**同时
                       装 xyz + 强度 + 颜色, 坐标是**绝对 UTM** (局部坐标 + utm_center)。
                       强度按原始测量值写(本数据 0~255), 不缩放。
  --utm_zone <n>       UTM 带号, 只用来往 las 里写 CRS 声明 (EPSG = 32600+n, WGS84 北半球)。
                       **不给就不写 CRS**, 坐标照样是绝对 UTM, 只是文件里不声明带号。
                       为什么不自动推: 一个点相对自己带中央经线的偏移总在 +-3 度内, 所以
                       **任何**带号都自洽 —— 只凭 (easting, northing) 数学上定不了带号,
                       猜错会把整张图放到地球上错误的位置且不报错。manifest 里也没有。
  --raw_export <x,y,size>  额外导出一块方框内的**原始点**: 不做体素滤波, 强度和点的分布
                        都是原始测量值。输出 <name>_{before,after}_raw.pcd (x y z intensity),
                        开了 --color 还会出 _raw_rgb.pcd。
                        **判路面漆重影就用这份**: 直接从原始帧来, 强度是逐点的原始测量值,
                        在看图工具里按强度着色即可 —— 不做任何提取/阈值, 免得判据本身出错。
                        为什么需要: 体素滤波把格内的点换成格心均值、强度也一起平均 ——
                        漆边缘的格子混了漆和沥青, 平均后强度被拉低、边界被抹圆, 点的分布
                        也从"扫描线"变成"规则格点"。判路面漆重影时这两件事都致命。
                        只能导一块: 全量原始点是一帧 20 万点 x 上万帧, 装不下。
  --calib_rounds <n>    按白化残差 rms 自校准各类约束 sigma 的轮数 (默认 0 = 不校准)
                        每轮: 解 -> 量各类 rms -> sigma *= rms (姿态是权重形式, w /= rms)
                        -> 从**同一初值**重解。收敛后各类 rms 都该 ≈ 1。
                        只校准测量项(帧间/视觉); 先验项(INS/姿态)冗余度≈0, 自校准会发散。
  --calib_priors <0|1>  1=连 INS/姿态先验也校准 (默认 0; 实测会一路收紧到把位姿钉死)

 增量建图 (多 session 依次并入, 第一个为基准):
  --incr                开启增量模式 (取代 --joint 的两 session 逻辑)
  --sess_order <a,b,c>  session 顺序 (逗号分隔的目录名); 默认按目录序
  --pose_store <dir>    位姿存档目录 (默认 <out>/poses)。**有存档 = 已优化, 本次跳过**
                        **强烈建议给一个独立于 --out 的持久目录**: --out 里是几十 GB 的
                        点云/las/dump, 反复实验时会被清掉; 而存档是增量建图的全部状态
                        (位姿 + 约束), 一起清掉就等于每次从零重跑。落在 --out 里会告警。
                        复用的三个条件 (逐 session 检查并打印原因):
                          1) <store>/<session>.csv 读得出来
                          2) base_utm 一致 —— 位姿是相对 utm_center 的局部坐标, center
                             不同就不在同一系里, 硬用会整体偏几百米
                          3) 帧集**按 pcd 路径 100% 命中** —— 差一帧就当待优化重做, 而不是
                             静默用半套位姿 (改 --drop_frames / --load_roi 都会让它失效)
  --redo <a,b>          强制重做这些 session (即使有存档)
  --list_done           只打印"哪些已优化/哪些待优化"然后退出, 不干活
  --load_g2o <file>     载入已有位姿图(需要同名 _index.csv)。里面的 session 视为已优化:
                        **它们的约束直接复用, 不重新构造 scan2submap / 帧间 GICP**,
                        但**位姿参与最终的联合优化**。这就是增量建图的状态文件 ——
                        每来一个新 session, 只对新 session 做若干次 scan2submap。
  --freeze_prev <0|1>   1 = 老 session 的位姿在联合优化里钉死 (默认 0 = 一起优化)

 同 session 回环 (独立于分窗逻辑的一路, 默认开):
  --no_loop             关掉回环
  --loop_dist <m>       空间距离上限 (默认 12)
  --loop_min_gap <n>    序号差下限 (默认 0 = 用 intra_win; 低于它的由分窗逻辑覆盖)
  --loop_step <m>       沿弧长每隔多少米开一次回环尝试 (默认 20)
  --loop_per_frame <n>  每次尝试最多留几条 (默认 2)
  --loop_max <n>        候选总数上限 (默认 6000)
  --loop_min_inlier <r> 回环的 inlier 门 (默认 0.6, 比窗口内的 0.2 重叠门严得多)
                        为什么要更严: 回环配错一条会把整段轨迹拽歪, 而窗口内配错只影响局部。
                        实测这批数据"空间<12m 且序号差>40"的帧对: jjst2 有 33559 对(同向
                        30470), 涉及 46% 的帧 —— 量级和现有窗口内约束(34991)相当,
                        而这些约束现在一条都没用上。
                        实测钉死有代价: ROI 里 jjst2<->jjst3 从 0.083 退化到 0.095 ——
                        新 session 只能单方面往老的上凑, 而两边各有误差时正确解是各让一半。
                        放开它几乎不要钱: 7.5 万条边/3.6 万参数的稀疏图解一次是秒级,
                        真正的成本在配准。
  --dump_cross <n>      **debug**: 每次 scan-to-submap 都存下来, 最多 n 帧 (默认 0=关)
                        输出到 <out>/cross/ :
                          r<区域>_submap.pcd            该区域的 target (前面所有 session 拼的)
                          r<区域>_nn<后>_g<改善>_...      _before.pcd / _after.pcd / .png
                        文件名把分数放在最前, 所以 ls 天然按质量排序, ls -r 从最差看起。
                        png 上下两行=配准前/后, 左右两列=全部点/地面以上, **看地面以上那行**。
  --drop_isolated <m>   连通半径 (默认 10, 0=关)。相距 <=m 的帧算相连, **只保留最大的
                        连通域**, 与主轨迹断开的整块全部丢掉。
                        判据是连通域而不是"半径内有没有别的帧": 实测 jjst2 的 clip _000000
                        有 54 帧停在起点原地不动(彼此间距 0.02m)、然后跳 118.9m 才接上主
                        轨迹(中间的 clip _000001 缺失) —— 这 54 帧离主图 85m, 在点云里就是
                        脱离主路线的一块, 而"最近邻>5m"对它原理上无效(最近邻是 0m)。
                        单帧孤立是连通域大小为 1 的特例, 一并覆盖。
                        !! 别把 m 给到 5: 实测正常关键帧间距中位 2.5~3.7m、最大 6.8m,
                           5m 会把正常轨迹切成碎块。每次运行都先打印实测分布, 照它调。
  --drop_frames <0|1>   **默认 0 = 一帧都不丢**。连通域/静止块的分析照常做并打印"本该丢
                        哪些", 但所有帧都参与优化 —— 分析只当诊断用, 不动数据。
                        给 1 才会真丢。要连分析都跳过, 用 --drop_isolated 0。
                        [实测] jjst2 的 clip _000000 那 54 帧 (原地不动、离主图 118.9m)
                        被 jjst5 以 3.5m 的距离覆盖过, 而 jjst5 是行驶中采的、拿得到真实
                        帧间约束。留着它 = 在**已被正确建图的位置**叠一份只有 INS 精度、
                        无人校正的副本 —— 那正是重影的配方。要留就要接受这一点。
  --drop_static <m>     连通域自身空间尺度小于这个就整块丢 (默认 5)。
                        **这才是真正的判据**: 实测三个 session 共 20 个连通域, 只有 jjst2 的
                        clip _000000 尺度为 0.0m (54 帧原地停着、离主图 118.9m, 就是导出点云里
                        那块脱离主路线的东西); 其余 19 块尺度 175~624m 全是真实路段, 只是
                        clip 之间缺数据。所以**不能**用"只保留最大连通域"(jjst3 那样会丢掉 79%
                        的真实数据)。路口等红灯的静止帧不会被误伤 —— 它们和行驶段连通, 属于
                        尺度几百米的大域。
  --opt_ext <n>         把**雷达外参 T_v_l 也放进 BA** 一起优化 (0=不动 1=只旋转 2=旋转+平移)
                        yaml 里的值当初值, 结束时打印优化前后的外参和变动量。
                        为此待优化量从 T_w_l 换成 T_w_v + 共享 T_v_l —— 直接优化 T_w_l 时
                        外参不可观测(外参转 δ 与每帧各自转 δ 是同一件事)。
                        平移默认不动: 杆臂在直路上和位置强耦合(病态)。
                        视觉项在这里有额外价值: 相机外参独立, 所以它能把"雷达外参错"和
                        "INS 航向偏置"分开 —— 前者会让点云投影错位, 后者不会。
  --yaw_fix_deg <deg>   给雷达外参补 yaw 修正, 右乘到 T_w_l (默认 0)
                        实测 WG_wuling 输入里有约 -1.87 度的偏航不一致: yaw(T_w_l) 与实际
                        行驶方向差 -2.19 度(直线段 std 0.53), 而外参文件只有 -0.32 度。
                        不修的话 BA 会把每块补丁绕各自锚定帧转 +2 度去补, 各块因此处于
                        不同的旋转规范系, 锚点之间就对不上了。
  --joint               选**旧路径** runJoint (两 session 一次性联合)。与 --incr 平行, 二者只能选一个
                        A/B 各自整段 BA -> 遍历所有重叠区域建跨 session 约束。**已被 --incr 取代**,
                        只留作两 session 的对照实验 —— --incr 是它的超集(单 session 也合法)。
                        -> 一起解一次。输出 A/B 优化前后的点云。
                        为什么要一次联合解而不是"先 BA 再挂上去": 各重叠区域之间必然互相
                        不完全自洽, 分两步做的话要么 B 的形状被拽变形, 要么约束被平均掉
                        谁也不满足 —— 主管线就是这么错的(满强度 kfba 让跨 session 从
                        0.074 变成 0.098, 同时把地板从 0.026 破坏到 0.042)。
  --load_roi <x,y,size> 只用位姿落在这个方框内的帧 (先在小范围试跑; --joint / --incr 都生效)
                        !! 它**改变帧集**, 因此会让位姿存档的复用判据不命中 —— 这是对的,
                        但意味着带 ROI 和不带 ROI 的两次运行不能共用一个 --pose_store。
  --intra_pair_dist <m> 同 session 内建 GICP 约束的最大帧间距离 (默认 12)
  --intra_win <n>       分窗处理的窗口帧数, 窗口间 50%% 重叠 (默认 40)
  --region_step <m>     沿共位段每隔多少米开一个重叠区域 (默认 150)
  --region_radius <m>   每个重叠区域的半径 (默认 40)
  --region_max_frames <n>  每个区域拼 submap 最多用多少参照帧 (默认 150, 0=不限)
                        配准**次数**只取决于新 session 的共位帧数, 与已有 session 数无关;
                        但单次配准的 target 会随 session 数线性膨胀(40m 区域现在 96~109 帧,
                        20 个 session 之后七百多帧/350 万点)。按到中心的距离排序后**按
                        session 轮转取**: 近处优先, 且每个 session 都有代表 —— 只按距离取
                        会让最近的那个 session 独占名额, 而多 session 的价值就在互相印证。
  --cross_sigma_r <rad> 跨 session 约束的旋转 sigma (默认 0.005)
  --cross_sigma_xy <m>  跨 session 约束的 sigma (默认 0.09)
                        **实测值**: 六处锚点上 B 配准后的 nn 是 0.070~0.103。
  --scan_overlap        只扫两条轨迹的共位情况然后退出, 不做配准。
                        **选 --center 之前先跑这个**: 两个 session 可能只是穿过同一片区域
                        而没走同一条路(实测某路口最近 A 帧也有 18m 远、航向差中位 92 度),
                        那里的"错位"其实是视角差异, 配准修不了也不该去修。
  --max_io_fail <r>     帧读取失败率上限 (默认 0.01; <=0 = 不检查)。超了就**中止且不写存档**。
                        [为什么] 有一次外挂盘中途掉线, loadPcd 连续失败 34566 次, 而每次失败
                        只是跳过那一帧继续跑, 没有汇总也没有非零退出码 —— 于是在数据大量缺失
                        的情况下"正常"跑完, 并把残缺数据配出来的约束写进了存档。增量建图里
                        污染过的位姿下一轮会被当成可信初值复用, 错误会一直传下去。
                        按 session 分段判定: 某个 session 全军覆没而别的正常时, 全局比例
                        可能还在阈值内。
  --frame_cache_mb <n>  帧缓存上限 (MB, 默认 1500; 0 = 关)。
                        loadFrame 每次都是"读 pcd + lzf 解压 + range 过滤 + 剔动态 + 体素滤波"
                        一整套, 而同一帧在一次运行里被读 12~16 次 (分窗 ~2 / 回环 submap ~3 /
                        跨session 区域 ~5 / 导出 2), 单次约 20 ms —— 配准阶段七成时间在这儿。
                        [实测单价] frame_voxel 0.05 -> 约 1 MB/帧; 0.03 -> 约 2.5 MB/帧,
                        所以按字节限容而不是帧数。导出阶段会主动清空(那里命中率为零)。
  --num_threads <n>     线程数 (默认 6)
)";
}

/// @brief 解析命令行。遇到未知参数直接失败并打帮助 —— **不静默忽略**:
///        拼错一个参数名而程序照跑, 得到的是"改了参数却没生效"的假结论。
/// @return false = 应当退出 (帮助/参数错)
static bool parse(int argc, char** argv, Opt& o) {
  bool has_center = false;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    const auto nd = [&](double& d) { d = std::stod(argv[++i]); };
    const auto ni = [&](int& d) { d = std::stoi(argv[++i]); };
    const auto ns = [&](std::string& d) { d = argv[++i]; };
    if (a == "-h" || a == "--help") { usage(); return false; }
    else if (a == "--params") { i++; }            // 已在 main 里预先读过, 这里只跳过它的值
    else if (a == "--save_params") ns(o.save_params);
    else if (a == "--root") ns(o.root);
    else if (a == "--out") ns(o.out);
    else if (a == "--data_mode") ns(o.data_mode);
    else if (a == "--pose_dir") ns(o.pose_dir);
    else if (a == "--attitude_from") ns(o.attitude_from);
    else if (a == "--lidar_dir") ns(o.lidar_dir);
    else if (a == "--pose_src") ns(o.pose_src);
    else if (a == "--sa") ni(o.sa);
    else if (a == "--sb") ni(o.sb);
    else if (a == "--radius") nd(o.radius);
    else if (a == "--b_radius") nd(o.b_radius);
    else if (a == "--b_max_dist") nd(o.b_max_dist);
    else if (a == "--b_max_dyaw") nd(o.b_max_dyaw);
    else if (a == "--frame_voxel") nd(o.frame_voxel);
    else if (a == "--min_range") nd(o.min_range);
    else if (a == "--max_range") nd(o.max_range);
    else if (a == "--submap_voxel") nd(o.submap_voxel);
    else if (a == "--tgt_voxel") nd(o.tgt_voxel);
    else if (a == "--iters") ni(o.iters);
    else if (a == "--no_fine") o.fine = false;
    else if (a == "--fine_corr") nd(o.fine_corr);
    else if (a == "--fine_iters") ni(o.fine_iters);
    else if (a == "--min_inlier") nd(o.min_inlier);
    else if (a == "--max_corr") nd(o.max_corr);
    else if (a == "--z_above") nd(o.z_above);
    else if (a == "--no_clamp") o.clamp_planar = false;
    else if (a == "--no_dyn_filter") o.dyn_filter = false;
    else if (a == "--dump_max") ni(o.dump_max);
    else if (a == "--num_threads") ni(o.num_threads);
    else if (a == "--frame_cache_mb") ni(o.frame_cache_mb);
    else if (a == "--max_io_fail") nd(o.max_io_fail);
    else if (a == "--scan_overlap") o.scan_overlap = true;
    else if (a == "--scan_step") nd(o.scan_step);
    else if (a == "--ba") o.ba = true;
    else if (a == "--ba_visual") { o.ba = true; o.ba_visual = true; }
    else if (a == "--ba_min_overlap") nd(o.ba_min_overlap);
    else if (a == "--ba_pair_voxel") nd(o.ba_pair_voxel);
    else if (a == "--ba_max_corr") nd(o.ba_max_corr);
    else if (a == "--ba_pair_iters") ni(o.ba_pair_iters);
    else if (a == "--dyn_ts_tol") nd(o.dyn_ts_tol);
    else if (a == "--ba_ins_xy") nd(o.ba_ins_xy);
    else if (a == "--ba_rel_t") nd(o.ba_rel_t);
    else if (a == "--ba_rel_r") nd(o.ba_rel_r);
    else if (a == "--ba_att_w") nd(o.ba_att_w);
    else if (a == "--ba_huber") nd(o.ba_huber);
    else if (a == "--ba_chi2") nd(o.ba_chi2);
    else if (a == "--ba_iters") ni(o.ba_iters);
    else if (a == "--ba_sigma_px") nd(o.ba_sigma_px);
    else if (a == "--vis_cams") ns(o.vis_cams);
    else if (a == "--yaw_fix_deg") nd(o.yaw_fix_deg);
    else if (a == "--opt_ext") ni(o.opt_ext);
    else if (a == "--color") o.color = true;
    else if (a == "--no_color") o.color = false;   // color 默认开, 需要能关
    else if (a == "--color_cams") ns(o.color_cams);
    else if (a == "--color_z_min") nd(o.color_z_min);
    else if (a == "--color_max_range") nd(o.color_max_range);
    else if (a == "--color_smear_max") nd(o.color_smear_max);
    else if (a == "--loop_min_arc") nd(o.loop_min_arc);
    else if (a == "--export_raw") ni(o.export_raw);
    else if (a == "--export_before") ni(o.export_before);
    else if (a == "--export") ni(o.do_export);
    else if (a == "--las") ni(o.las);
    else if (a == "--utm_zone") ni(o.utm_zone);
    else if (a == "--color_ground_h") nd(o.color_ground_h);
    else if (a == "--color_occ_tol") nd(o.color_occ_tol);
    else if (a == "--color_occ_px") ni(o.color_occ_px);
    else if (a == "--color_drop") ni(o.color_drop);
    else if (a == "--raw_export") {
      const std::string v = argv[++i];
      const auto c1 = v.find(','), c2 = v.find(',', c1 + 1);
      o.raw_x = std::stod(v.substr(0, c1));
      o.raw_y = std::stod(v.substr(c1 + 1, c2 - c1 - 1));
      o.raw_size = std::stod(v.substr(c2 + 1));
    }
    else if (a == "--calib_rounds") ni(o.calib_rounds);
    else if (a == "--calib_priors") ni(o.calib_priors);
    else if (a == "--incr") o.incr = true;
    else if (a == "--pose_store") ns(o.pose_store);
    else if (a == "--redo") ns(o.redo);
    else if (a == "--sess_order") ns(o.sess_order);
    else if (a == "--list_done") o.list_done = true;
    else if (a == "--dump_cross") ni(o.dump_cross);
    else if (a == "--drop_isolated") nd(o.drop_isolated);
    else if (a == "--drop_static") nd(o.drop_static);
    else if (a == "--drop_frames") ni(o.drop_frames);
    else if (a == "--load_g2o") ns(o.load_g2o);
    else if (a == "--freeze_prev") ni(o.freeze_prev);
    else if (a == "--no_loop") o.loop = false;
    else if (a == "--loop_dist") nd(o.loop_dist);
    else if (a == "--loop_min_gap") ni(o.loop_min_gap);
    else if (a == "--loop_step") nd(o.loop_step);
    else if (a == "--loop_per_frame") ni(o.loop_per_frame);
    else if (a == "--loop_max") ni(o.loop_max);
    else if (a == "--loop_min_inlier") nd(o.loop_min_inlier);
    else if (a == "--loop_min_inlier_rev") nd(o.loop_min_inlier_rev);
    else if (a == "--cross_ba") ni(o.cross_ba);
    else if (a == "--cross_ba_edges") ni(o.cross_ba_edges);
    else if (a == "--cross_ba_max") ni(o.cross_ba_max);
    else if (a == "--cross_src_win") ni(o.cross_src_win);
    else if (a == "--b_bidir") ni(o.b_bidir);
    else if (a == "--cross_per_sess") ni(o.cross_per_sess);
    else if (a == "--nms_loop") nd(o.nms_loop);
    else if (a == "--nms_cross") nd(o.nms_cross);
    else if (a == "--nms_win_deltas") ns(o.nms_win_deltas);
    else if (a == "--loop_submap") ni(o.loop_submap);
    else if (a == "--loop_submap_radius") nd(o.loop_submap_radius);
    else if (a == "--loop_ba") ni(o.loop_ba);
    else if (a == "--loop_sigma_t") nd(o.loop_sigma_t);
    else if (a == "--loop_sigma_r") nd(o.loop_sigma_r);
    else if (a == "--loop_anti_w") nd(o.loop_anti_w);
    else if (a == "--dump_loop") ni(o.dump_loop);
    else if (a == "--dump_pcd") ni(o.dump_pcd);
    else if (a == "--loop_cluster_dist") nd(o.loop_cluster_dist);
    else if (a == "--loop_cluster_max") ni(o.loop_cluster_max);
    else if (a == "--joint") o.joint = true;
    else if (a == "--intra_pair_dist") nd(o.intra_pair_dist);
    else if (a == "--intra_win") ni(o.intra_win);
    else if (a == "--region_step") nd(o.region_step);
    else if (a == "--region_radius") nd(o.region_radius);
    else if (a == "--region_max_frames") ni(o.region_max_frames);
    else if (a == "--cross_sigma_xy") nd(o.cross_sigma_xy);
    else if (a == "--cross_sigma_r") nd(o.cross_sigma_r);
    else if (a == "--load_roi") {
      const std::string v = argv[++i];
      const auto c1 = v.find(','), c2 = v.find(',', c1 + 1);
      o.load_roi_x = std::stod(v.substr(0, c1));
      o.load_roi_y = std::stod(v.substr(c1 + 1, c2 - c1 - 1));
      o.load_roi_size = std::stod(v.substr(c2 + 1));
    }
    else if (a == "--center") {
      const std::string v = argv[++i];
      const auto c = v.find(',');
      o.cx = std::stod(v.substr(0, c));
      o.cy = std::stod(v.substr(c + 1));
      has_center = true;
    } else {
      std::cerr << "未知参数: " << a << "\n";
      usage();
      return false;
    }
  }
  // --center 只有单锚点模式需要 (它要一个具体位置); --joint/--incr 自己遍历全程
  if (!has_center && !o.joint && !o.incr && !o.scan_overlap) {
    std::cerr << "必须给 --center x,y  (先跑主程序看日志里的'轨迹包围盒'来选)\n";
    usage();
    return false;
  }
  if (o.b_radius <= 0) o.b_radius = o.radius;
  return true;
}

// -----------------------------------------------------------------------------
// 小工具 (与 incremental_g2o 里的同名函数行为一致, 这里为了独立编译复制一份)
// -----------------------------------------------------------------------------
/// @brief 体素滤波: 每格取均值(位置和强度), 原地替换输入。
/// @param ints 强度; 长度与 pts 相同时才一起处理, 否则忽略并清空
/// @param res  体素边长; <=0 时直接返回不做任何事
/// @note 格心用**均值**而不是取其中一个点 —— 均值把同一表面的多次采样合成一个更准的点,
///       这也是"点数减少 = 内部更齐"这个代理指标成立的原因。
/// @note 坐标先加 2^20 再取 21 位掩码, 所以可表达范围是 +-1048576 格; 超出的点被丢弃。
static void voxelDownsample(std::vector<Eigen::Vector4d>& pts, std::vector<double>& ints, double res) {
  if (res <= 0.0 || pts.empty()) return;
  const bool has_i = ints.size() == pts.size();
  const double inv = 1.0 / res;
  struct Acc { Eigen::Vector3d s{0, 0, 0}; double i = 0; int n = 0; };
  std::map<std::uint64_t, Acc> grid;
  constexpr std::int64_t kOff = 1 << 20;
  constexpr std::uint64_t kMask = (1ull << 21) - 1;
  for (std::size_t k = 0; k < pts.size(); k++) {
    const std::int64_t cx = static_cast<std::int64_t>(std::floor(pts[k].x() * inv)) + kOff;
    const std::int64_t cy = static_cast<std::int64_t>(std::floor(pts[k].y() * inv)) + kOff;
    const std::int64_t cz = static_cast<std::int64_t>(std::floor(pts[k].z() * inv)) + kOff;
    if (cx < 0 || cy < 0 || cz < 0) continue;
    const std::uint64_t key = (static_cast<std::uint64_t>(cx) & kMask) |
                              ((static_cast<std::uint64_t>(cy) & kMask) << 21) |
                              ((static_cast<std::uint64_t>(cz) & kMask) << 42);
    auto& acc = grid[key];
    acc.s += pts[k].head<3>();
    if (has_i) acc.i += ints[k];
    acc.n++;
  }
  std::vector<Eigen::Vector4d> op;
  std::vector<double> oi;
  op.reserve(grid.size());
  for (const auto& [k, acc] : grid) {
    const Eigen::Vector3d m = acc.s / acc.n;
    op.emplace_back(m.x(), m.y(), m.z(), 1.0);
    if (has_i) oi.push_back(acc.i / acc.n);
  }
  pts.swap(op);
  ints.swap(oi);
}

/// @brief 读一帧: 去车身/远点 -> (可选)剔动态 -> 体素滤波。顺序固定。
// -----------------------------------------------------------------------------
// 帧缓存 (LRU, 按字节限容)
//
// 为什么需要: loadFrame 每次调用都是完整的一遍 —— 读 pcd + lzf 解压 + range 过滤 +
// 读 3dod 标注剔动态 + 体素滤波。而它有 20 个调用点, 同一帧在一次运行里被反复读:
//   分窗 BA ~2 次(窗口 40/步长 20, 50% 重叠) | 回环 submap ~3 次(693 端点 x 11 帧)
//   跨session 区域 ~5 次(138 区域 x ~100 参照帧) | 导出 2 次
// 实测每帧 12~16 次, 单次约 20 ms(数据在外挂 USB 盘上) —— 配准阶段七成时间花在这儿。
//
// 访问有很强的局部性: 窗口、回环 submap、跨session 区域的工作集都在几十到一百多帧,
// 所以几百帧的 LRU 就能吃掉大部分重复。
//
// 三个设计点:
//   1) **按字节限容, 不按帧数** —— 每帧的点数随 frame_voxel 变 (0.05 时约 2.5 万点/1 MB,
//      0.03 时约 6 万点/2.5 MB), 写死帧数会让内存占用随参数漂移。
//   2) **命中时拷贝一份出去** —— 调用方拿到点云后会就地变换/传给 gtsam_points, 不能给引用。
//      拷 1 MB 约 0.1 ms, 比读盘的 20 ms 便宜两个数量级。
//   3) **读盘在锁外** —— 否则并行区里所有线程会在第一个未命中上排队。
//      代价是偶尔两个线程同时读同一帧(浪费一次), 比串行化划算得多。
//
// 导出阶段会主动清空(clear): 那里每帧只顺序读一次, 命中率为零, 留着纯粹和体素累积争内存。
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// 帧读取的成败统计 + 写存档前的守门
//
// 为什么必须有: 2026-08-28 那次增量跑, 外挂盘中途掉线(整个挂载点消失), loadPcd 连续失败
// 34566 次。而每次失败只是"跳过这一帧"继续跑, 没有任何汇总, 最后**退出码 0** ——
// 于是在数据大量缺失的情况下"正常"跑完 21 分钟, 并把用残缺数据配出来的约束和位姿写进了存档。
// 增量建图里这尤其致命: 污染过的位姿下一轮会被当作可信初值复用, 错误一直传下去。
//
// 按 session 分段统计(不是全局比例): 某个 session 全军覆没、而别的正常时, 全局比例可能
// 还在阈值内。中止时**不写存档** —— 宁可这一轮白跑, 也不能留下一份看不出问题的坏存档。
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// 并行配准循环的进度打印。
//
// 为什么带 ETA 而不只是百分比: 这些循环动辄几千次配准、几十分钟, 看到"还剩多久"才知道
// 要不要等。ETA 用**已完成部分的实际速率**外推, 比按帧数估的靠谱。
//
// 输出用 \n 而不是 \r: 实际运行都是 nohup 重定向到日志文件, \r 会把整个文件糊成一行。
// printf 对同一个 FILE* 是加锁的, 所以并行调用不会把一行撕开。
// -----------------------------------------------------------------------------
struct Progress {
  Progress(const char* tag, const char* what, std::size_t total, std::size_t step = 100)
      : tag_(tag), what_(what), total_(total), step_(step ? step : 100),
        t0_(std::chrono::steady_clock::now()) {}
  void tick() {
    const std::size_t d = done_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (d % step_ != 0 && d != total_) return;
    const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    const double eta = d ? el * static_cast<double>(total_ - d) / static_cast<double>(d) : 0.0;
    if (el < 20.0 && d != total_) return;   // 太快的循环不值得刷屏
    printf("      %s %s %zu/%zu (%.0f%%)  已用 %.1f min, 预计剩 %.1f min\n", tag_, what_, d,
           total_, 100.0 * static_cast<double>(d) / static_cast<double>(total_), el / 60.0,
           eta / 60.0);
    std::fflush(stdout);
  }

private:
  const char* tag_;
  const char* what_;
  std::size_t total_, step_;
  std::atomic<std::size_t> done_{0};
  std::chrono::steady_clock::time_point t0_;
};

struct IoStat {
  std::atomic<std::size_t> ok{0}, fail{0};
  std::size_t mark_ok = 0, mark_fail = 0;
  void bump(bool good) { (good ? ok : fail).fetch_add(1, std::memory_order_relaxed); }
  void mark() {
    mark_ok = ok.load();
    mark_fail = fail.load();
  }
  std::size_t segOk() const { return ok.load() - mark_ok; }
  std::size_t segFail() const { return fail.load() - mark_fail; }
};
static IoStat g_io;

/// @brief 写存档之前叫一次。读失败率超阈值就打印原因并**退出**, 不写任何存档。
/// @param seg true = 只看上次 mark() 以来的这一段 (单个 session)
static void guardIo(double max_ratio, const char* where, bool seg) {
  const std::size_t f = seg ? g_io.segFail() : g_io.fail.load();
  const std::size_t k = seg ? g_io.segOk() : g_io.ok.load();
  if (f + k == 0) return;
  const double r = static_cast<double>(f) / static_cast<double>(f + k);
  if (f) printf("  [帧读取] 成功 %zu, **失败 %zu** (%.2f%%)  [%s]\n", k, f, 100.0 * r, where);
  if (max_ratio <= 0 || r <= max_ratio) return;
  printf("\n!! 帧读取失败率 %.2f%% 超过上限 %.2f%% (--max_io_fail) —— **中止, 不写存档**\n",
         100.0 * r, 100.0 * max_ratio);
  printf("   最常见的原因是数据盘掉线或文件被移走。先确认 root 下的数据完整, 再重跑。\n");
  printf("   已有的存档没有被改动, 可以直接接着增量。\n");
  std::fflush(stdout);
  std::exit(2);
}

class FrameCache {
public:
  void setCapacity(std::size_t bytes) {
    std::lock_guard<std::mutex> lk(mu_);
    cap_ = bytes;
    evict();
  }
  /// @brief 命中则把点云**拷贝**到 out 并返回 true
  bool get(const std::string& k, ialign::PcdCloud& out, long& n_dyn) {
    std::lock_guard<std::mutex> lk(mu_);
    if (cap_ == 0) return false;
    const auto it = idx_.find(k);
    if (it == idx_.end()) { ++miss_; return false; }
    lru_.splice(lru_.begin(), lru_, it->second);   // 提到最近使用
    out = it->second->cloud;
    n_dyn = it->second->n_dyn;
    ++hit_;
    return true;
  }
  void put(const std::string& k, const ialign::PcdCloud& v, long n_dyn) {
    const std::size_t sz = bytesOf(v);
    std::lock_guard<std::mutex> lk(mu_);
    if (cap_ == 0 || sz > cap_) return;
    const auto it = idx_.find(k);
    if (it != idx_.end()) {                        // 并发读同一帧时会走到这里
      bytes_ -= bytesOf(it->second->cloud);
      lru_.erase(it->second);
      idx_.erase(it);
    }
    lru_.push_front(Entry{k, v, n_dyn});
    idx_[k] = lru_.begin();
    bytes_ += sz;
    evict();
  }
  void clear() {
    std::lock_guard<std::mutex> lk(mu_);
    lru_.clear();
    idx_.clear();
    bytes_ = 0;
  }
  void report(const char* tag) {
    std::lock_guard<std::mutex> lk(mu_);
    const std::size_t tot = hit_ + miss_;
    if (tot == 0 || cap_ == 0) return;
    printf("  [帧缓存 %s] 命中 %zu / %zu (%.1f%%), 省下约 %zu 次读盘; 当前 %zu 帧 / %.2f GB (上限 %.2f GB)\n",
           tag, hit_, tot, 100.0 * hit_ / tot, hit_, lru_.size(),
           bytes_ / 1073741824.0, cap_ / 1073741824.0);
  }

private:
  struct Entry {
    std::string key;
    ialign::PcdCloud cloud;
    long n_dyn;
  };
  static std::size_t bytesOf(const ialign::PcdCloud& c) {
    return c.points.size() * sizeof(Eigen::Vector4d) + c.intensities.size() * sizeof(double) + 96;
  }
  void evict() {                                   // 调用者已持锁
    while (bytes_ > cap_ && !lru_.empty()) {
      bytes_ -= bytesOf(lru_.back().cloud);
      idx_.erase(lru_.back().key);
      lru_.pop_back();
    }
  }
  std::mutex mu_;
  std::list<Entry> lru_;
  std::unordered_map<std::string, std::list<Entry>::iterator> idx_;
  std::size_t bytes_ = 0, cap_ = 0, hit_ = 0, miss_ = 0;
};
static FrameCache g_fcache;

static bool loadFrame(const std::string& path, const Opt& o, ialign::PcdCloud& out, long* n_dyn = nullptr) {
  {
    long nd = 0;
    if (g_fcache.get(path, out, nd)) {             // 命中: 结果与重新算一遍逐位相同
      if (n_dyn) *n_dyn = nd;
      return !out.points.empty();
    }
  }
  ialign::PcdCloud raw;
  if (!ialign::loadPcd(path, raw, true)) {
    g_io.bump(false);
    return false;
  }
  g_io.bump(true);
  const bool has_i = raw.intensities.size() == raw.points.size();
  const double r2max = o.max_range > 0 ? o.max_range * o.max_range : 0.0;
  const double r2min = o.min_range > 0 ? o.min_range * o.min_range : 0.0;
  out.points.clear();
  out.intensities.clear();
  for (std::size_t i = 0; i < raw.points.size(); i++) {
    const double r2 = raw.points[i].head<3>().squaredNorm();
    if (r2min > 0 && r2 < r2min) continue;
    if (r2max > 0 && r2 > r2max) continue;
    out.points.push_back(raw.points[i]);
    if (has_i) out.intensities.push_back(raw.intensities[i]);
  }
  // 动态点必须在**体素滤波之前**剔 —— 滤波会把车上的点和背景点平均进同一格,
  // 之后再删只会留下被污染的格心。
  if (o.dyn_filter) {
    const long n = ialign::removeDynPoints(path, o.dyn_ts_tol, out.points, out.intensities);
    if (n_dyn) *n_dyn = n;
  }
  voxelDownsample(out.points, out.intensities, o.frame_voxel);
  g_fcache.put(path, out, n_dyn ? *n_dyn : 0);
  return !out.points.empty();
}

/// @brief 给点云算法向量和协方差 (GICP/VGICP 的必需输入)。
/// @param nt 线程数; 在**已经并行到帧**的地方一律传 1, 否则会嵌套并行把线程数翻倍
/// @note k=10 近邻。glim 的 CloudCovarianceEstimation 内部做了正则化(把最小特征值抬起来),
///       所以平面上的点得到的是"扁"协方差 —— GICP 的 plane-to-plane 行为就来自这里。
/// @brief 读一帧但**不做体素滤波**: 只去车身/远点 + (可选)剔动态。
///
/// 为什么要单独一个: loadFrame 末尾的 voxelDownsample 会把格内的点换成**格心均值**,
/// 强度也一起平均 —— 于是导出的点云既不是原始测量位置, 强度也被抹平了。
/// 判路面漆重影时这两件事都致命: 漆边缘的格子混了漆和沥青, 平均后强度被拉低,
/// 边界被抹圆; 点的分布也从"扫描线"变成"规则格点", 看不出真实采样结构。
static bool loadFrameRaw(const std::string& path, const Opt& o, ialign::PcdCloud& out) {
  ialign::PcdCloud raw;
  if (!ialign::loadPcd(path, raw, true)) {
    g_io.bump(false);
    return false;
  }
  g_io.bump(true);
  const bool has_i = raw.intensities.size() == raw.points.size();
  const double r2max = o.max_range > 0 ? o.max_range * o.max_range : 0.0;
  const double r2min = o.min_range > 0 ? o.min_range * o.min_range : 0.0;
  out.points.clear();
  out.intensities.clear();
  for (std::size_t i = 0; i < raw.points.size(); i++) {
    const double r2 = raw.points[i].head<3>().squaredNorm();
    if (r2min > 0 && r2 < r2min) continue;
    if (r2max > 0 && r2 > r2max) continue;
    out.points.push_back(raw.points[i]);
    if (has_i) out.intensities.push_back(raw.intensities[i]);
  }
  if (o.dyn_filter) ialign::removeDynPoints(path, o.dyn_ts_tol, out.points, out.intensities);
  return !out.points.empty();
}

static void addCovs(const gtsam_points::PointCloudCPU::Ptr& c,
                    const ialign::CloudCovarianceEstimation& ce, int nt) {
  if (c->size() == 0) return;
  const int k = 10;
  gtsam_points::KdTree2<gtsam_points::PointCloud> tree(c);
  std::vector<int> nb(c->size() * k);
#pragma omp parallel for num_threads(nt) schedule(guided, 8)
  for (std::int64_t i = 0; i < static_cast<std::int64_t>(c->size()); i++) {
    std::vector<size_t> ki(k);
    std::vector<double> kd(k);
    tree.knn_search(c->points[i].data(), k, ki.data(), kd.data());
    std::copy(ki.begin(), ki.end(), nb.begin() + i * k);
  }
  std::vector<Eigen::Vector4d> nr;
  std::vector<Eigen::Matrix4d> cv;
  ce.estimate(c->points_storage, nb, nr, cv);
  c->add_normals(nr);
  c->add_covs(cv);
}

/// @brief 6-DOF 配准结果 -> 只保留平面自由度: x/y/yaw 取配准值, z/roll/pitch 取初值。
///
/// 为什么这么做: 平坦道路上 LiDAR 在垂直方向退化(地面是一个大平面, 竖直方向的最近邻
/// 代价很平), 配准出的 z/roll/pitch 噪声大; 而 INS 的 roll/pitch 来自**重力对齐**,
/// 长期无漂移, 比配准可靠。所以把这三个自由度交还给初值。
///
/// @note **本函数不读任何重力数据**。上面说的"重力对齐"是在解释 INS 的 roll/pitch
///       为什么可信, 不是说这里用了重力向量。这里只是把两个旋转按 ZYX 欧拉角分解再
///       重组, 隐含假设**世界系 +Z 朝上** —— 位姿在 UTM 局部系(ENU)里, 这一点成立。
///       显式用重力方向的是 submap_ceres.hpp 里的 AttitudePriorG (它接一个 g0 参数,
///       把姿态误差投影到 g 上分解, 与坐标约定无关, 更严谨)。
/// @note `Rz(yaw_reg)*Ry(pitch_ini)*Rx(roll_ini)` **不严格等价于**"只绕重力轴转" ——
///       车辆 roll/pitch 只有几度时两者差别可忽略, 但这是个近似。
/// @note ZYX 分解在 pitch = +-90 度处奇异, 对车辆不构成问题。
static Eigen::Isometry3d clampPlanar(const Eigen::Isometry3d& reg, const Eigen::Isometry3d& ini) {
  const auto rpy = [](const Eigen::Matrix3d& R, double& r, double& p, double& y) {
    r = std::atan2(R(2, 1), R(2, 2));
    p = -std::asin(std::max(-1.0, std::min(1.0, R(2, 0))));
    y = std::atan2(R(1, 0), R(0, 0));
  };
  double r1, p1, y1, r0, p0, y0;
  rpy(reg.linear(), r1, p1, y1);
  rpy(ini.linear(), r0, p0, y0);
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = (Eigen::AngleAxisd(y1, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(p0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(r0, Eigen::Vector3d::UnitX())).toRotationMatrix();
  out.translation() << reg.translation().x(), reg.translation().y(), ini.translation().z();
  return out;
}

// -----------------------------------------------------------------------------
// 覆盖门 —— 不加这个, nn 会被"B 帧覆盖到而 submap 没覆盖到"的区域主导。
//
// B 的单帧有 60m 量程, 而 A 的 submap 只覆盖邻域附近的走廊。超出覆盖的源点找不到最近邻,
// 被记成搜索上限(3m), 直接把中位数抬起来 —— 实测本工具第一版因此把 0.7 量级的错位
// 报成 1.1~1.4m。这与 pcd_diff --mutual 是同一个问题, 那边修过, 这里第一版又犯了一次。
// 做法: 按 2m 体素记下 submap 的占据格, 只统计落在占据格里的源点。
// -----------------------------------------------------------------------------
/// @brief 把一个点量化成体素键 (每轴 21 位, 三轴打包进 int64)。
/// @note 用于占据集合, 不用于滤波 —— 这里没有 +offset, 负坐标靠 & 0x1FFFFF 回绕。
///       同一个 cell 下自洽即可, 不要求可逆。
static std::int64_t occKey(const Eigen::Vector3d& p, double cell) {
  const auto q = [&](double v) { return static_cast<std::int64_t>(std::floor(v / cell)) & 0x1FFFFF; };
  return (q(p.x()) << 42) | (q(p.y()) << 21) | q(p.z());
}
/// @brief 建"哪些体素里有点"的**有序去重**列表, 供 std::binary_search 查询。
/// @return 升序、无重复的体素键。查询是 O(log n), 建表是 O(n log n)。
static std::vector<std::int64_t> buildOcc(const std::vector<Eigen::Vector4d>& pts, double cell) {
  std::vector<std::int64_t> k;
  k.reserve(pts.size());
  for (const auto& p : pts) k.push_back(occKey(p.head<3>(), cell));
  std::sort(k.begin(), k.end());
  k.erase(std::unique(k.begin(), k.end()), k.end());
  return k;
}

/// @brief 只用**地面以上**的点算中位最近邻。全部点会被路面主导, 对横向错位近乎全盲
///        (实测低估约一倍: 0.146 vs 0.207 / 0.163 vs 0.369)。
static double aboveGroundNN(const gtsam_points::PointCloud& tgt, const ialign::GroundMap& gm,
                           const gtsam_points::KdTree& tree, const gtsam_points::PointCloud& src,
                           const Eigen::Isometry3d& T, double z_above, int max_q, double cap,
                           const std::vector<std::int64_t>* occ = nullptr, double occ_cell = 2.0,
                           long* n_used = nullptr, long* n_out = nullptr) {
  (void)tgt;
  std::vector<double> d;
  long nout = 0;
  const std::size_t step = std::max<std::size_t>(1, src.size() / std::max(1, max_q));
  for (std::size_t i = 0; i < src.size(); i += step) {
    const Eigen::Vector4d p = T * src.points[i];
    const double zg = gm.at(p.x(), p.y());
    if (zg < -1e17 || p.z() - zg < z_above) continue;
    // 覆盖门: submap 在这个 2m 格里没有点, 说明这块本来就不在 A 的覆盖范围内,
    // 拿它的最近邻去算等于在量"覆盖差异", 不是量对齐。
    if (occ && !std::binary_search(occ->begin(), occ->end(), occKey(p.head<3>(), occ_cell))) {
      nout++;
      continue;
    }
    std::size_t ki = 0;
    double kd = 0;
    if (tree.knn_search(p.data(), 1, &ki, &kd) > 0) {
      const double dd = std::sqrt(kd);
      if (dd <= cap) d.push_back(dd);
    }
  }
  if (n_used) *n_used = static_cast<long>(d.size());
  if (n_out) *n_out = nout;
  if (d.size() < 20) return -1.0;
  std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
  return d[d.size() / 2];
}

/// @brief 写二进制 PCD。给了 ints 且长度与 v 一致就带上 intensity 字段, 否则只写 xyz。
/// @note 强度是判**路面漆重影**的正确工具: 它和距离是同一束激光测出来的, 零投影误差;
///       相机上色在掠射的地面上会被放大到几十厘米(见 colorizeFrame 的入射角门)。
static void savePts(const fs::path& p, const std::vector<Eigen::Vector4d>& v,
                    const std::vector<double>& ints = {}) {
  ialign::savePcdBinary(p.string(), v, ints.size() == v.size() ? ints : std::vector<double>{});
}

// -----------------------------------------------------------------------------
// 点云上色 (相机内外参投影)
//
// 只做纯针孔投影 —— 这批数据的 distCoeffs 全为零(图像已去畸变), 所以这是准确的,
// 不是近似。图像和点云同名同时刻, 也不需要运动补偿。
//
// 两件必须做对的事:
//   1. **遮挡**。没有深度检查时, 被前景挡住的远处点会拿到前景的颜色 —— 表现为墙面上
//      糊着一片车的颜色, 而且这种错色会随视角变化, 看起来极像重影。这里用一个粗
//      深度缓冲(默认 4px 一格)存每格最近深度, 比它深出 color_occ_tol 的点不上色。
//   2. **选相机**。一个点常同时落在两三个相机里。取"离主点最近"的那个 —— 边缘处
//      角分辨率低、且相邻相机重叠区的曝光差最大, 取中心能让拼缝最不明显。
// -----------------------------------------------------------------------------
using Rgb = Eigen::Matrix<std::uint8_t, 3, 1>;

struct ColorStat {
  long n_pts = 0, n_col = 0, n_occ = 0, n_img_miss = 0, n_frame_noimg = 0;
  long n_smear = 0;   // 被入射角门挡掉的次数 (掠射的地面点)
};

/// @brief 给一帧点云(雷达系)上色。rgb 与 pts 等长; ok[i]=0 表示这个点没上到色。
static void colorizeFrame(const std::string& pcd_path, const std::vector<Eigen::Vector4d>& pts,
                          const std::vector<ialign::CamModel>& cams,
                          const Eigen::Isometry3d& T_v_l, const Opt& o,
                          std::vector<Rgb>& rgb, std::vector<char>& ok, ColorStat& st) {
  const std::size_t n = pts.size();
  rgb.assign(n, Rgb(70, 70, 70));   // 没上到色的点涂深灰, 而不是丢掉 —— 丢掉会留下空洞,
  ok.assign(n, 0);                  // 那种空洞看图时很容易被误读成"这里没扫到"。
  st.n_pts += static_cast<long>(n);
  const double r2max = o.color_max_range > 0 ? o.color_max_range * o.color_max_range : 0.0;

  std::vector<float> best_score(n, 1e9f);
  std::vector<Eigen::Vector3f> uvz(n);
  int n_img = 0;
  // 每个点在**车体系**下的高度 (车体原点在地面, 所以这就是离地高度)。
  // 入射角门要用它区分"地面点"和"竖直面上的点" —— 两者对同样的像素误差敏感度差一个量级。
  std::vector<double> pv(n);
  {
    const Eigen::Matrix4d Mv = T_v_l.matrix();
    for (std::size_t i = 0; i < n; i++) pv[i] = (Mv * pts[i]).z();
  }

  for (const auto& cam : cams) {
    const auto ip = ialign::imagePathFor(pcd_path, cam.name, o.data_mode);
    if (ip.empty()) { st.n_img_miss++; continue; }
    const cv::Mat img = cv::imread(ip.string(), cv::IMREAD_COLOR);
    if (img.empty() || img.cols != cam.width || img.rows != cam.height) { st.n_img_miss++; continue; }
    n_img++;
    const Eigen::Matrix4d M = (cam.T_v_c.inverse() * T_v_l).matrix();

    // --- 投影 + 建深度缓冲 ---
    const int bw = cam.width / o.color_occ_px + 1, bh = cam.height / o.color_occ_px + 1;
    std::vector<float> buf(static_cast<std::size_t>(bw) * bh, 1e9f);
    for (std::size_t i = 0; i < n; i++) {
      uvz[i].z() = -1.0f;
      if (r2max > 0 && pts[i].head<3>().squaredNorm() > r2max) continue;
      const Eigen::Vector4d pc = M * pts[i];
      if (pc.z() < o.color_z_min) continue;
      // ---- 入射角门 ----
      // 相机系: x 右 y 下 z 前。地面点 y>0 (在相机下方)。射线打到高度 y=h 的平面上时
      // 深度 z = h*fy/(v-cy)  =>  |dz/dv| = z^2/(h*fy), 这就是**一个像素造成的地面位移**。
      // 掠射时放大极大: h=1.8m、fy=775, z=30m 时 1 像素 = 0.65m。
      // 只对接近地面的点判(用车体系高度识别); 竖直面上 1 像素只对应 z/fx, 不需要门。
      if (o.color_smear_max > 0 && pv[i] < o.color_ground_h) {
        const double h = std::abs(pc.y());
        const double smear = h > 1e-3 ? pc.z() * pc.z() / (cam.fy * h) : 1e9;
        if (smear > o.color_smear_max) { st.n_smear++; continue; }
      }
        const double u = cam.fx * pc.x() / pc.z() + cam.cx;
        const double v = cam.fy * pc.y() / pc.z() + cam.cy;
        // 丢掉落在图像边缘附近的点: 距离边缘需至少 margin_px 像素
        const int margin_px = 2;
        if (u < margin_px || v < margin_px || u >= cam.width - margin_px ||
          v >= cam.height - margin_px) continue;
      uvz[i] = Eigen::Vector3f(static_cast<float>(u), static_cast<float>(v),
                               static_cast<float>(pc.z()));
      float& b = buf[static_cast<std::size_t>(static_cast<int>(v) / o.color_occ_px) * bw +
                     static_cast<int>(u) / o.color_occ_px];
      if (uvz[i].z() < b) b = uvz[i].z();
    }
    // --- 取色 (过遮挡门 + 与已有相机比"离主点多近") ---
    for (std::size_t i = 0; i < n; i++) {
      if (uvz[i].z() < 0) continue;
      const int bu = static_cast<int>(uvz[i].x()) / o.color_occ_px;
      const int bv = static_cast<int>(uvz[i].y()) / o.color_occ_px;
      if (uvz[i].z() > buf[static_cast<std::size_t>(bv) * bw + bu] + o.color_occ_tol) {
        st.n_occ++;
        continue;
      }
      const float du = static_cast<float>((uvz[i].x() - cam.cx) / cam.width);
      const float dv = static_cast<float>((uvz[i].y() - cam.cy) / cam.height);
      const float s = du * du + dv * dv;
      if (s >= best_score[i]) continue;
      best_score[i] = s;
      const cv::Vec3b& c = img.at<cv::Vec3b>(static_cast<int>(uvz[i].y()),
                                             static_cast<int>(uvz[i].x()));
      rgb[i] = Rgb(c[2], c[1], c[0]);   // OpenCV 是 BGR
      ok[i] = 1;
    }
  }
  if (n_img == 0) st.n_frame_noimg++;
  for (std::size_t i = 0; i < n; i++) st.n_col += ok[i];
}

/// @brief 边攒边并格的体素累加器。
///
/// 为什么要它: 导出整段 session 时, "先把所有帧的点堆进一个 vector 再滤波"要 2500 帧
/// x 3 万点 x 32 字节 ≈ 2.4GB, 加上颜色和中间副本能到 5GB —— 这个项目已经被 OOM
/// 看守终止过四次。这里直接往哈希格里累加和, 内存只和**输出点数**成正比,
/// 且结果与一次性滤波**逐点相同**(都是格内均值)。
struct VoxelAcc {
  double res = 0.15;
  /// n = 落进这一格的点数; nc = 其中**上到色**的点数 (颜色只在这 nc 个点上取均值)
  struct Acc { Eigen::Vector3d s{0, 0, 0}; Eigen::Vector3d c{0, 0, 0}; double si = 0;
               int n = 0, nc = 0, ni = 0; };
  std::unordered_map<std::uint64_t, Acc> grid;

  /// @param ok 与 pts 等长; ok[k]=0 表示这个点没投到任何图像上 -> 不参与颜色均值。
  ///           传 nullptr 表示全部算有色。
  void add(const std::vector<Eigen::Vector4d>& pts, const std::vector<Rgb>* rgb,
           const std::vector<char>* ok = nullptr, const std::vector<double>* ints = nullptr) {
    const bool has_i = ints && ints->size() == pts.size();
    const double inv = 1.0 / res;
    constexpr std::int64_t kOff = 1 << 20;
    constexpr std::uint64_t kMask = (1ull << 21) - 1;
    for (std::size_t k = 0; k < pts.size(); k++) {
      const std::int64_t cx = static_cast<std::int64_t>(std::floor(pts[k].x() * inv)) + kOff;
      const std::int64_t cy = static_cast<std::int64_t>(std::floor(pts[k].y() * inv)) + kOff;
      const std::int64_t cz = static_cast<std::int64_t>(std::floor(pts[k].z() * inv)) + kOff;
      if (cx < 0 || cy < 0 || cz < 0) continue;
      const std::uint64_t key = (static_cast<std::uint64_t>(cx) & kMask) |
                                ((static_cast<std::uint64_t>(cy) & kMask) << 21) |
                                ((static_cast<std::uint64_t>(cz) & kMask) << 42);
      auto& a = grid[key];
      a.s += pts[k].head<3>();
      a.n++;
      if (has_i) { a.si += (*ints)[k]; a.ni++; }
      // 只把**真上到色**的点算进颜色均值。之前把灰色占位 (70,70,70) 也平均进去,
      // 一格里 1 个有色 + 3 个灰点就会被冲淡成接近灰。
      if (rgb && (!ok || (*ok)[k])) {
        a.c += (*rgb)[k].cast<double>();
        a.nc++;
      }
    }
  }
  /// @brief 出两份: 全部体素(位置) + 其中有颜色的那些(位置和颜色)。
  ///
  /// 为什么要分开: 纯 xyz 那份是量重影/厚度的**基准**, 必须始终完整 —— 单相机只能覆盖
  /// 约一半的点, 如果按"没上到色就丢"去砍它, 导出的点云就变成了前视视场的形状,
  /// 前后两轮的测量基准不一致, 数字不可比。
  /// @param drop_uncolored true = 未上色的体素不进 rgb 那份(留空洞但颜色干净);
  ///                       false = 未上色的涂深灰后也进 rgb 那份
  /// @param oa 与 op **等长**的颜色 (未上色的给黑)。LAS 一个文件同时装 xyz+强度+颜色,
  ///           不像 PCD 要分两份, 所以需要这份对齐的颜色。
  void finish(std::vector<Eigen::Vector4d>& op, std::vector<Eigen::Vector4d>& cp,
              std::vector<Rgb>& oc, bool with_rgb, bool drop_uncolored,
              std::vector<double>* oi = nullptr, std::vector<Rgb>* oa = nullptr) {
    op.clear();
    cp.clear();
    oc.clear();
    op.reserve(grid.size());
    if (oi) { oi->clear(); oi->reserve(grid.size()); }
    if (oa) { oa->clear(); oa->reserve(grid.size()); }
    for (const auto& [k, a] : grid) {
      const Eigen::Vector3d m = a.s / a.n;
      op.emplace_back(m.x(), m.y(), m.z(), 1.0);
      if (oi) oi->push_back(a.ni ? a.si / a.ni : 0.0);
      if (oa) {
        if (a.nc > 0) {
          const Eigen::Vector3d c = a.c / a.nc;
          oa->emplace_back(static_cast<std::uint8_t>(std::lround(c.x())),
                           static_cast<std::uint8_t>(std::lround(c.y())),
                           static_cast<std::uint8_t>(std::lround(c.z())));
        } else {
          oa->emplace_back(0, 0, 0);
        }
      }
      if (!with_rgb) continue;
      if (a.nc > 0) {
        const Eigen::Vector3d c = a.c / a.nc;
        cp.emplace_back(m.x(), m.y(), m.z(), 1.0);
        oc.emplace_back(static_cast<std::uint8_t>(std::lround(c.x())),
                        static_cast<std::uint8_t>(std::lround(c.y())),
                        static_cast<std::uint8_t>(std::lround(c.z())));
      } else if (!drop_uncolored) {
        cp.emplace_back(m.x(), m.y(), m.z(), 1.0);
        oc.emplace_back(70, 70, 70);
      }
    }
    grid.clear();
  }
};

/// @brief 带颜色的体素滤波: 位置和 RGB 都在格内取均值。
static void voxelDownsampleRgb(std::vector<Eigen::Vector4d>& pts, std::vector<Rgb>& rgb,
                               double res) {
  if (res <= 0.0 || pts.empty()) return;
  const double inv = 1.0 / res;
  struct Acc { Eigen::Vector3d s{0, 0, 0}; Eigen::Vector3d c{0, 0, 0}; int n = 0; };
  std::unordered_map<std::uint64_t, Acc> grid;
  grid.reserve(pts.size() / 2);
  constexpr std::int64_t kOff = 1 << 20;
  constexpr std::uint64_t kMask = (1ull << 21) - 1;
  for (std::size_t k = 0; k < pts.size(); k++) {
    const std::int64_t cx = static_cast<std::int64_t>(std::floor(pts[k].x() * inv)) + kOff;
    const std::int64_t cy = static_cast<std::int64_t>(std::floor(pts[k].y() * inv)) + kOff;
    const std::int64_t cz = static_cast<std::int64_t>(std::floor(pts[k].z() * inv)) + kOff;
    if (cx < 0 || cy < 0 || cz < 0) continue;
    const std::uint64_t key = (static_cast<std::uint64_t>(cx) & kMask) |
                              ((static_cast<std::uint64_t>(cy) & kMask) << 21) |
                              ((static_cast<std::uint64_t>(cz) & kMask) << 42);
    auto& a = grid[key];
    a.s += pts[k].head<3>();
    a.c += rgb[k].cast<double>();
    a.n++;
  }
  std::vector<Eigen::Vector4d> op;
  std::vector<Rgb> oc;
  op.reserve(grid.size());
  oc.reserve(grid.size());
  for (const auto& [k, a] : grid) {
    const Eigen::Vector3d m = a.s / a.n, c = a.c / a.n;
    op.emplace_back(m.x(), m.y(), m.z(), 1.0);
    oc.emplace_back(static_cast<std::uint8_t>(std::lround(c.x())),
                    static_cast<std::uint8_t>(std::lround(c.y())),
                    static_cast<std::uint8_t>(std::lround(c.z())));
  }
  pts.swap(op);
  rgb.swap(oc);
}

// =============================================================================
// 联合模式
//
// 流程: A/B 各自整段 BA -> 遍历所有重叠区域建跨 session 约束 -> 一起解一次。
//
// 关键实现选择: **A、B 的位姿拼成同一个参数向量** (B 的下标偏移 n_a)。于是
//   - "单独 BA" = 只放该 session 的约束, 解一次;
//   - "联合优化" = 两个 session 的约束 + 跨 session 约束, 解一次;
// 两者共用同一个解算器 (optimizeSubmapCeresExt), 跨 session 约束就是普通的帧间相对位姿,
// 只是两端分属不同 session。不需要"锚点"这种特殊结构。
//
// 为什么必须一次联合解, 不能"先 BA 再挂上去": 各重叠区域之间必然互相不完全自洽(实测
// 锚间不一致 0.016~0.341m), 分两步做要么 B 的形状被拽变形, 要么约束被平均掉谁也不满足。
// 主管线就是这么错的 —— 满强度 kfba 让跨 session 从 0.074 变成 0.098, 地板从 0.026 到 0.042。
// =============================================================================
/// @brief 四维非极大抑制: **两端都**落在 R 内才算重复, 同组内只留质量最好的那条。
///
/// 为什么是"两端都": A<->B 和 A<->B+1 的中点只差半个帧距, 用中点当键能并掉它们, 但两个
/// 几何上完全不同的对也可能共享中点 —— 中点是有损的。用两端的位置对才不会错杀
/// **同一地点的不同重访** (实测四维比二维多留 121/458 条回环、245/1640 条跨session)。
///
/// @param pi,pj 每条边两端的世界位置 (2D)
/// @param qual  质量, **越小越优先** (用 nn 或残差; 没有就传 -inlier)
/// @param grp   分组; **只在同组内抑制**。跨 session 的组不同, 所以 A 与 session1 的 B
///              建了约束后, session2 里 B 附近的 C 仍然会建 A<->C —— 多 session 互相
///              印证不能被去重吃掉。
/// @return 与输入等长的保留掩码
/// @note 用 R 大小的网格给 i 端建桶, 只和邻桶里已接受的比 —— 抑制要求 i 端在 R 内,
///       所以邻桶足够, 避免 O(n^2)。
static std::vector<char> nms4d(const std::vector<Eigen::Vector2d>& pi,
                               const std::vector<Eigen::Vector2d>& pj,
                               const std::vector<double>& qual, const std::vector<int>& grp,
                               double R) {
  const std::size_t n = pi.size();
  std::vector<char> keep(n, 1);
  if (R <= 0.0 || n == 0) return keep;
  std::vector<std::size_t> ord(n);
  for (std::size_t k = 0; k < n; k++) ord[k] = k;
  std::stable_sort(ord.begin(), ord.end(), [&](std::size_t a, std::size_t b) {
    if (grp[a] != grp[b]) return grp[a] < grp[b];
    return qual[a] < qual[b];
  });
  const double R2 = R * R;
  // (组, 格) -> 该格里已接受边的下标
  std::map<std::tuple<int, long, long>, std::vector<std::size_t>> cell;
  for (const std::size_t k : ord) {
    const long cx = static_cast<long>(std::floor(pi[k].x() / R));
    const long cy = static_cast<long>(std::floor(pi[k].y() / R));
    bool dup = false;
    for (long dx = -1; dx <= 1 && !dup; dx++) {
      for (long dy = -1; dy <= 1 && !dup; dy++) {
        const auto it = cell.find({grp[k], cx + dx, cy + dy});
        if (it == cell.end()) continue;
        for (const std::size_t a : it->second) {
          if ((pi[a] - pi[k]).squaredNorm() <= R2 && (pj[a] - pj[k]).squaredNorm() <= R2) {
            dup = true;
            break;
          }
        }
      }
    }
    if (dup) {
      keep[k] = 0;
      continue;
    }
    cell[{grp[k], cx, cy}].push_back(k);
  }
  return keep;
}

/// @brief 把 "1,5,20,40" 解析成允许的帧号差集合; 空串返回空集 = 不限制。
static std::set<int> parseDeltas(const std::string& c) {
  std::set<int> out;
  std::size_t p = 0;
  while (p <= c.size() && !c.empty()) {
    const auto q = c.find(',', p);
    const std::string t = c.substr(p, q == std::string::npos ? std::string::npos : q - p);
    if (!t.empty()) out.insert(std::atoi(t.c_str()));
    if (q == std::string::npos) break;
    p = q + 1;
  }
  return out;
}

/// @brief 打印一批边沿弧长的分布均匀性 —— "均匀化"要看的是这个, 不是总条数。
static void reportSpread(const char* tag, const char* what, const std::vector<double>& arc_i) {
  if (arc_i.empty()) {
    printf("  [%s] %s: 0 条\n", tag, what);
    return;
  }
  std::vector<double> a = arc_i;
  std::sort(a.begin(), a.end());
  std::vector<double> gap;
  for (std::size_t k = 1; k < a.size(); k++) gap.push_back(a[k] - a[k - 1]);
  std::map<long, int> cellc;
  for (const double v : a) cellc[static_cast<long>(v / 50.0)]++;
  std::vector<int> cv;
  for (const auto& [c, n] : cellc) cv.push_back(n);
  std::sort(cv.begin(), cv.end());
  double gm = 0, z = 0;
  if (!gap.empty()) {
    std::vector<double> g2 = gap;
    std::sort(g2.begin(), g2.end());
    gm = g2[g2.size() / 2];
    for (const double v : gap) z += (v < 0.5) ? 1 : 0;
    z = 100.0 * z / gap.size();
  }
  printf("  [%s] %s: %zu 条 | 相邻边弧长间隔 中位 %.1fm, <0.5m 占 %.0f%% | "
         "50m 格 有边 %zu 个, 每格 中位 %d max %d\n",
         tag, what, a.size(), gm, z, cellc.size(), cv[cv.size() / 2], cv.back());
}

struct IntraCon {
  std::vector<std::tuple<int, int, Eigen::Isometry3d>> rel;   // 下标是**全局**下标
  /// 与 rel 等长的种类标记: 0=窗口内 2=同向回环 3=反向回环。
  /// 用来给白化 rms 分组和逐条给 sigma —— 混在一起的话两百条回环会被三万条窗口边淹掉。
  std::vector<int> kind;
  std::vector<ialign::VisReprojSpecExt> vis;
  std::size_t n_cand = 0, n_big = 0;
  double inl_med = -1;
  ialign::VisualStats vst;          // 视觉对应的建立过程统计 (各道门剔了多少)
  std::size_t vis_dup = 0;          // 因窗口重叠被去掉的重复视觉对应
};

/// @brief 同 session 内的 GICP(+视觉) 约束。分窗处理, 窗口间 50% 重叠 ——
///        全部帧的点云同时留在内存是不可行的(2500 帧 x 3 万点 x 192 字节 = 14GB)。
static IntraCon buildIntra(const ialign::SessionData& S, const std::vector<int>& idx, int gofs,
                          const Opt& o, const ialign::CloudCovarianceEstimation& ce,
                          const std::vector<ialign::CamModel>& cams, const char* tag) {
  IntraCon out;
  std::set<std::pair<int, int>> vis_edge;   // 已经出过视觉对应的边 (全局下标)
  const int n = static_cast<int>(idx.size());
  // 只建这些帧号差的约束。**在候选阶段就拦**, 所以省掉的是配准时间(成本大头) ——
  // 实测同session边里 Δ>=2 占 92%, 而它们说的是同一段局部几何重复几十遍。
  const std::set<int> dlt = parseDeltas(o.nms_win_deltas);
  if (!dlt.empty()) {
    printf("  [%s] 窗口内只建帧号差 ∈ {", tag);
    for (const int d : dlt) printf("%d ", d);
    printf("} 的约束 (其余在候选阶段就跳过, 不配准)\n");
  }
  const int win = std::max(4, o.intra_win);
  const int step = std::max(1, win / 2);
  std::vector<double> inls;
  // **在候选阶段就跨窗口查重**。窗口 50% 重叠, 重叠区里的对本来会在两个窗口各配一次 VGICP,
  // 而后面按 (i,j) 去重时第二次的结果直接丢弃 —— 配准是耗时大头, 那部分算力纯属白花。
  // 提前查重靠的是**初值**: 用不到配准结果, 所以能在花掉 VGICP 之前就跳过。
  //
  // 语义与原来一致: 两次都保留"先出现的那个窗口"的结果。之所以行为不变, 是因为同一对在
  // 两个窗口里的初值(T_w_l 的相对位姿)、点云、体素图分辨率完全相同, 那几道门和 VGICP 都是
  // 确定性的 —— 第一个窗口过不了门/配准失败的对, 第二个窗口同样过不了。
  std::set<std::pair<int, int>> seen_pair;
  std::size_t n_dup_skip = 0;
  // 进度按**窗口**报而不是按窗口内的配准: 每个窗口只有二十来个候选(nms_win_deltas 筛过),
  // 放在里面会让 125 个窗口各刷一行。
  Progress prog_win(tag, "分窗 BA (窗口)", static_cast<std::size_t>((n + step - 1) / step), 5);
  for (int b0 = 0; b0 < n; b0 += step) {
    const int b1 = std::min(n, b0 + win);
    if (b1 - b0 < 2) break;
    const int m = b1 - b0;
    std::vector<ialign::PcdCloud> pc(m);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t k = 0; k < m; k++) loadFrame(S.frames[idx[b0 + k]].pcd_path, o, pc[k]);
    std::vector<gtsam_points::PointCloudCPU::Ptr> cl(m);
    std::vector<gtsam_points::GaussianVoxelMap::Ptr> vm(m);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t k = 0; k < m; k++) {
      if (pc[k].points.empty()) continue;
      auto c = std::make_shared<gtsam_points::PointCloudCPU>();
      c->add_points(pc[k].points);
      addCovs(c, ce, 1);
      auto v = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.ba_pair_voxel);
      v->insert(*c);
      cl[k] = c;
      vm[k] = v;
    }
    // 只建窗口内的对; 窗口 50% 重叠保证跨窗口的近邻对不会漏
    std::vector<std::pair<int, int>> cand;
    for (int i = 0; i < m; i++) {
      if (!vm[i]) continue;
      for (int j = i + 1; j < m; j++) {
        if (!cl[j]) continue;
        if (!dlt.empty() && !dlt.count(j - i)) continue;   // 帧号差不在允许集里 -> 不配准
        const Eigen::Isometry3d T0 =
          S.frames[idx[b0 + i]].T_w_l.inverse() * S.frames[idx[b0 + j]].T_w_l;
        if (T0.translation().norm() > o.intra_pair_dist) continue;
        if (gtsam_points::overlap_auto(vm[i], cl[j], T0) < o.ba_min_overlap) continue;
        if (!seen_pair.insert({b0 + i, b0 + j}).second) {   // 上一个窗口已经配过这一对
          n_dup_skip++;
          continue;
        }
        cand.emplace_back(i, j);
      }
    }
    std::vector<Eigen::Isometry3d> rr(cand.size());
    std::vector<char> ok(cand.size(), 0);
    std::vector<double> iv(cand.size(), -1);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t c = 0; c < static_cast<std::int64_t>(cand.size()); c++) {
      const int i = cand[c].first, j = cand[c].second;
      const Eigen::Isometry3d T0 =
        S.frames[idx[b0 + i]].T_w_l.inverse() * S.frames[idx[b0 + j]].T_w_l;
      gtsam::Values vals;
      vals.insert(0, gtsam::Pose3(T0.matrix()));
      gtsam::NonlinearFactorGraph g;
      auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vm[i], cl[j]);
      g.add(f);
      try {
        gtsam_points::LevenbergMarquardtExtParams lm;
        lm.setMaxIterations(o.ba_pair_iters);
        vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
      } catch (const std::exception&) {
        continue;
      }
      const Eigen::Isometry3d T1(vals.at<gtsam::Pose3>(0).matrix());
      iv[c] = f->inlier_fraction();
      if ((T1.translation() - T0.translation()).norm() > o.ba_max_corr) continue;
      rr[c] = T1;
      ok[c] = 1;
    }
    // 去重: 窗口重叠会让同一对被算两次, 用 set 拦住 (重复约束等于把权重加倍)
    static thread_local std::set<std::pair<int, int>> dummy;
    for (std::size_t c = 0; c < cand.size(); c++) {
      if (iv[c] >= 0) inls.push_back(iv[c]);
      out.n_cand++;
      if (!ok[c]) {
        if (iv[c] >= 0) out.n_big++;
        continue;
      }
      out.rel.emplace_back(gofs + b0 + cand[c].first, gofs + b0 + cand[c].second, rr[c]);
      out.kind.push_back(0);   // 0 = 窗口内帧间
    }

    // ---- 视觉重投影对应 (与 GICP 用同一批 cand 边) ----
    // 必须在窗口内做: buildVisualCorrs 要拿到帧点云(拼深度子图)和图像, 而点云只在
    // 本窗口活着。下标是窗口局部的, 存进 out.vis 时换成全局下标。
    if (!cams.empty() && !cand.empty()) {
      std::vector<Eigen::Isometry3d> Tw(m);
      for (int k = 0; k < m; k++) Tw[k] = S.frames[idx[b0 + k]].T_w_l;
      ialign::VisualOpts vo;
      vo.enable = true;
      vo.sigma_px = o.ba_sigma_px;
      vo.dump_max = 0;
      const auto img_of = [&](int k, int ci) {
        return ialign::imagePathFor(S.frames[idx[b0 + k]].pcd_path, cams[ci].name, o.data_mode);
      };
      const auto cloud_of = [&](int k) {
        return (k >= 0 && k < m) ? pc[k].points : std::vector<Eigen::Vector4d>();
      };
      const auto pcd_of = [&](int k) {
        return (k >= 0 && k < m) ? S.frames[idx[b0 + k]].pcd_path : std::string();
      };
      ialign::VisualStats vst;
      const auto vc = ialign::buildVisualCorrs(cand, Tw, Tw, cams, img_of, cloud_of, pcd_of,
                                              o.data_mode, S.T_v_l, vo, o.num_threads, vst, {});
      out.vst.edges += vst.edges;
      out.vst.raw += vst.raw;
      out.vst.drop_desc += vst.drop_desc;
      out.vst.drop_fmat += vst.drop_fmat;
      out.vst.drop_nodepth += vst.drop_nodepth;
      out.vst.drop_gate += vst.drop_gate;
      out.vst.kept += vst.kept;
      out.vst.err_sum += vst.err_sum;
      for (const auto& c : vc) {
        const int gi = gofs + b0 + c.i, gj = gofs + b0 + c.j;
        // 窗口 50% 重叠 -> 落在重叠区里的边会被算两遍。同一条边的视觉对应留第一次,
        // 否则那条边的视觉权重凭空翻倍(与 rel 的 (i,j) 去重是同一个道理)。
        if (!vis_edge.insert({gi, gj}).second) { out.vis_dup++; continue; }
        ialign::VisReprojSpecExt vs;
        vs.i = gi;
        vs.j = gj;
        vs.p_i = c.p_i;
        vs.T_c_v = cams[c.cam].T_v_c.inverse().matrix();
        vs.fx = cams[c.cam].fx;
        vs.fy = cams[c.cam].fy;
        vs.cx = cams[c.cam].cx;
        vs.cy = cams[c.cam].cy;
        vs.u = c.u;
        vs.v = c.v;
        out.vis.push_back(vs);
      }
    }
    prog_win.tick();
    if (b0 + win >= n) break;
  }
  // 去重 (i,j) 的**自检**: 候选阶段已经跨窗口查过重, 这里应该一条都删不掉。
  // 留着是因为它便宜(一次遍历), 而"重复约束等于把同一个测量的权重加倍"这种错很难从结果看出来。
  {
    std::set<std::pair<int, int>> seen;
    std::vector<std::tuple<int, int, Eigen::Isometry3d>> uniq;
    std::vector<int> uk;
    for (std::size_t u = 0; u < out.rel.size(); u++) {
      const auto& [i, j, T] = out.rel[u];
      if (!seen.insert({i, j}).second) continue;
      uniq.emplace_back(i, j, T);
      uk.push_back(u < out.kind.size() ? out.kind[u] : 0);
    }
    const std::size_t n_removed = out.rel.size() - uniq.size();
    out.rel.swap(uniq);
    out.kind.swap(uk);
    if (n_dup_skip || n_removed) {
      printf("  [%s] 窗口重叠去重: 候选阶段跳过 %zu 对(省掉这么多次 VGICP)%s\n", tag, n_dup_skip,
             n_removed ? " !! 配准后还删掉了 " : "");
      if (n_removed) printf("      %zu 条 —— 说明候选阶段的查重有漏\n", n_removed);
    }
  }
  std::sort(inls.begin(), inls.end());
  out.inl_med = inls.empty() ? -1 : inls[inls.size() / 2];
  printf("  [%s] 帧=%d  GICP 候选=%zu 采纳=%zu (修正过大剔=%zu)  inlier中位=%.3f\n", tag, n,
         out.n_cand, out.rel.size(), out.n_big, out.inl_med);
  if (!cams.empty()) {
    printf("  [%s] 视觉: 边=%ld 原始匹配=%ld -> 描述子剔=%ld 极几何剔=%ld 无深度剔=%ld gate剔=%ld"
           " -> **保留=%zu** (窗口重复去掉 %zu)  建边时误差均值=%.2f px\n",
           tag, out.vst.edges, out.vst.raw, out.vst.drop_desc, out.vst.drop_fmat,
           out.vst.drop_nodepth, out.vst.drop_gate, out.vis.size(), out.vis_dup,
           out.vst.kept ? out.vst.err_sum / out.vst.kept : -1.0);
  }
  return out;
}

// =============================================================================
// 同 session 内的回环约束 —— **独立于分窗逻辑的一路**
//
// buildIntra 只在滑动窗口(40 帧, 50% 重叠)内建对, 也就是要求两帧**序号相近**。同一个
// 地点隔很久再经过一次时, 两帧空间上挨着但落在不同窗口里, 一条约束都不会建。
//
// 实测这批数据里这种"空间近、时序远"的帧对有多少 (距离<12m 且序号差>40):
//   jjst2  33559 对(其中同向 30470), 涉及 1124 帧 = 全 session 的 46%
//   jjst3  11891 对(同向 11758)   jjst4  24291(23990)   jjst5  51845(49740)
// 同向占 91%, 说明这些不是对面车道逆行经过, 而是真的沿同一方向又走了一遍(绕环线、
// 来回跑同一条路), 配准条件很好。
//
// 为什么这件事重要: 一直量到的"同 session 内部误差 0.072m"很可能主要就是回环没闭合 ——
// 2.4km 的轨迹只有窗口内 40 帧的局部约束, 全局形状靠 INS 先验维持, 重访同一地点时两次
// 位姿必然差几厘米到十几厘米, 而这个误差从来没有任何约束去消除。而跨 session 配准的
// target 就是前面 session 的点云, target 自己糊 0.072, 跨 session 不可能好过它。
//
// 三个实现上的关键选择:
//   1. **候选只用位姿筛, 不碰点云** —— 3~5 万个候选对如果都要加载点云才能判断, 光 IO
//      就不可接受。
//   2. **并行到"对", 每个线程自己加载那两帧** —— 回环对的两端在时序上很远, 不可能像
//      buildIntra 那样一个窗口的点云共用。内存 = 线程数 x 2 帧, 与候选总数无关。
//   3. **限流 + 更严的门** —— 回环配错的代价比窗口内高得多(会把整段轨迹拽歪)。沿弧长
//      每 loop_step 米才开一次尝试, 每帧最多留 loop_per_frame 条, inlier 门也比窗口内
//      的高。回环约束的价值在于**闭合**, 不在于数量。
// =============================================================================
/// @brief 一批帧的**局部联合 BA**: 帧间 GICP 测相对位姿 + 以传入位姿为先验, 锚定一帧。
///
/// 用在两处: 跨 session 的目标端 (邻域内前面所有 session 的帧一起解, 目的是消掉它们在
/// 这个位置的相互不一致) 和源端 (当前帧 +-n 帧)。与窗口 BA 同一套逻辑, 只是范围小。
///
/// 先验取**传入的位姿本身**(不是原始 INS): 这些 session 已经优化过, 它们的位姿携带了全局
/// 对齐信息, 用 INS 当先验会把它们拽回去、毁掉全局一致性。这里只想做**局部**的相对调整,
/// 所以以当前估计为先验 + 锚定一帧就是正确的正则化。
///
/// @param T      进: 初值兼先验; 出: 优化后 (失败时不变)
/// @param anchor 哪一帧固定 (定局部规范系)
/// @param clouds 出: 各帧的点云 (省得建 submap 时再读一遍盘)
/// @param disp   出: 各帧被挪动的距离中位数 —— 它就是"这一片原本有多不一致"
/// @return true = 求解成功
static bool localBA(const std::vector<std::string>& paths, std::vector<Eigen::Isometry3d>& T,
                    int anchor, const Opt& o, const ialign::CloudCovarianceEstimation& ce,
                    std::vector<ialign::PcdCloud>* clouds, double* disp) {
  const int m = static_cast<int>(paths.size());
  if (m < 3) return false;
  std::vector<ialign::PcdCloud> pc(m);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
  for (std::int64_t k = 0; k < m; k++) loadFrame(paths[k], o, pc[k]);
  std::vector<gtsam_points::PointCloudCPU::Ptr> cl(m);
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> vm(m);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
  for (std::int64_t k = 0; k < m; k++) {
    if (pc[k].points.empty()) continue;
    auto c = std::make_shared<gtsam_points::PointCloudCPU>();
    c->add_points(pc[k].points);
    addCovs(c, ce, 1);
    auto v = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.ba_pair_voxel);
    v->insert(*c);
    cl[k] = c;
    vm[k] = v;
  }
  // ---- 帧间 GICP: 与 buildIntra 完全同样的做法和门 ----
  std::vector<std::pair<int, int>> cand;
  for (int i = 0; i < m; i++) {
    if (!vm[i]) continue;
    for (int j = i + 1; j < m; j++) {
      if (!cl[j]) continue;
      const Eigen::Isometry3d Ti = T[i].inverse() * T[j];
      if (Ti.translation().norm() > o.intra_pair_dist) continue;
      if (gtsam_points::overlap_auto(vm[i], cl[j], Ti) < o.ba_min_overlap) continue;
      cand.emplace_back(i, j);
    }
  }
  std::vector<Eigen::Isometry3d> rr(cand.size());
  std::vector<char> ok(cand.size(), 0);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
  for (std::int64_t c = 0; c < static_cast<std::int64_t>(cand.size()); c++) {
    const int i = cand[c].first, j = cand[c].second;
    const Eigen::Isometry3d Ti = T[i].inverse() * T[j];
    gtsam::Values vals;
    vals.insert(0, gtsam::Pose3(Ti.matrix()));
    gtsam::NonlinearFactorGraph g;
    auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vm[i], cl[j]);
    g.add(f);
    try {
      gtsam_points::LevenbergMarquardtExtParams lm;
      lm.setMaxIterations(o.ba_pair_iters);
      vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
    } catch (const std::exception&) {
      continue;
    }
    const Eigen::Isometry3d T1(vals.at<gtsam::Pose3>(0).matrix());
    if ((T1.translation() - Ti.translation()).norm() > o.ba_max_corr) continue;
    rr[c] = T1;
    ok[c] = 1;
  }
  std::vector<std::tuple<int, int, Eigen::Isometry3d>> rel;
  for (std::size_t c = 0; c < cand.size(); c++) {
    if (ok[c]) rel.emplace_back(cand[c].first, cand[c].second, rr[c]);
  }
  if (clouds) *clouds = std::move(pc);
  if (rel.size() < 2) return false;
  ialign::SubmapCeresOpts co;
  co.sigma_ins_xy = o.ba_ins_xy;
  co.sigma_rel_t = o.ba_rel_t;
  co.sigma_rel_r = o.ba_rel_r;
  co.attitude_w = o.ba_att_w;
  co.huber = o.ba_huber;
  co.iters = o.ba_iters;
  co.report = false;
  std::vector<Eigen::Isometry3d> Tout;
  const auto st = ialign::optimizeSubmapCeres(Tout, T, rel, Eigen::Vector3d::UnitZ(),
                                             std::max(0, std::min(m - 1, anchor)), co);
  if (!st.ok || Tout.size() != static_cast<std::size_t>(m)) return false;
  if (disp) {
    std::vector<double> d;
    for (int k = 0; k < m; k++) d.push_back((Tout[k].translation() - T[k].translation()).norm());
    std::sort(d.begin(), d.end());
    *disp = d[d.size() / 2];
  }
  T = Tout;
  return true;
}

/// @brief 给一个回环端点算出它的局部 submap 成员帧和**优化后**的相对位姿。
///
/// 为什么要先 BA 再合并: 直接用 INS 相对位姿在 +-5 帧(约 +-25m 行驶)上拼, 实测目标端
/// submap 的自身重影中位是 0.143 m —— 而配准精度的上限就是 target 自身的清晰度,
/// 也就是说回环约束再怎么配也好不过 0.14, 比帧间可重复性 0.035 差 4 倍。
/// 这里用的是**与窗口 BA 同一套逻辑**(帧间 GICP 测相对位姿 + INS 位置/重力姿态先验 +
/// Ceres), 只是范围小、锚在中心帧上, 所以结果直接就是中心帧雷达系下的相对位姿。
///
/// @param c0 中心帧 (在 idx 空间里的下标); 它是锚, 相对位姿以它为原点
/// @return (成员帧下标, T_c0_member); 失败或 BA 不收敛时退回 INS 相对位姿
static std::vector<std::pair<int, Eigen::Isometry3d>> loopSubmapPoses(
  const ialign::SessionData& S, const std::vector<int>& idx, int c0,
  const std::vector<Eigen::Vector2d>& P, const Opt& o,
  const ialign::CloudCovarianceEstimation& ce, double* ba_disp = nullptr) {
  const int n = static_cast<int>(idx.size());
  const int w = std::max(0, o.loop_submap);
  std::vector<int> mem;
  for (int d = -w; d <= w; d++) {
    const int c1 = c0 + d;
    if (c1 < 0 || c1 >= n) continue;
    // 距离门: 红灯前停着的那一段会把十几帧堆在同一个位置, 只增 IO 不增信息
    if (d != 0 && (P[c1] - P[c0]).norm() > o.loop_submap_radius) continue;
    mem.push_back(c1);
  }
  const auto ins_rel = [&](int c1) {
    return S.frames[idx[c0]].T_w_l.inverse() * S.frames[idx[c1]].T_w_l;
  };
  std::vector<std::pair<int, Eigen::Isometry3d>> out;
  if (mem.size() < 3 || !o.loop_ba) {
    for (const int c1 : mem) out.emplace_back(c1, ins_rel(c1));
    return out;
  }
  const int m = static_cast<int>(mem.size());
  int ci = 0;
  for (int k = 0; k < m; k++) {
    if (mem[k] == c0) ci = k;
  }
  // ---- 帧间 GICP: 与 buildIntra 里完全同样的做法 ----
  std::vector<gtsam_points::PointCloudCPU::Ptr> cl(m);
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> vm(m);
  for (int k = 0; k < m; k++) {
    ialign::PcdCloud pc;
    if (!loadFrame(S.frames[idx[mem[k]]].pcd_path, o, pc)) continue;
    auto c = std::make_shared<gtsam_points::PointCloudCPU>();
    c->add_points(pc.points);
    addCovs(c, ce, 1);
    auto v = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.ba_pair_voxel);
    v->insert(*c);
    cl[k] = c;
    vm[k] = v;
  }
  std::vector<Eigen::Isometry3d> T0(m);
  for (int k = 0; k < m; k++) T0[k] = S.frames[idx[mem[k]]].T_w_l;
  std::vector<std::tuple<int, int, Eigen::Isometry3d>> rel;
  for (int a = 0; a < m; a++) {
    if (!vm[a]) continue;
    for (int b = a + 1; b < m; b++) {
      if (!cl[b]) continue;
      const Eigen::Isometry3d Ti = T0[a].inverse() * T0[b];
      if (Ti.translation().norm() > o.intra_pair_dist) continue;
      if (gtsam_points::overlap_auto(vm[a], cl[b], Ti) < o.ba_min_overlap) continue;
      gtsam::Values vals;
      vals.insert(0, gtsam::Pose3(Ti.matrix()));
      gtsam::NonlinearFactorGraph g;
      auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vm[a],
                                                                      cl[b]);
      g.add(f);
      try {
        gtsam_points::LevenbergMarquardtExtParams lm;
        lm.setMaxIterations(o.ba_pair_iters);
        vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
      } catch (const std::exception&) {
        continue;
      }
      const Eigen::Isometry3d T1(vals.at<gtsam::Pose3>(0).matrix());
      // 修正过大 = 大概率配错, 宁可不要这条 (与 buildIntra 一致)
      if ((T1.translation() - Ti.translation()).norm() > o.ba_max_corr) continue;
      rel.emplace_back(a, b, T1);
    }
  }
  if (rel.size() < 2) {
    for (const int c1 : mem) out.emplace_back(c1, ins_rel(c1));
    return out;
  }
  ialign::SubmapCeresOpts co;
  co.sigma_ins_xy = o.ba_ins_xy;
  co.sigma_rel_t = o.ba_rel_t;      // 这里是**窗口内**帧对, 0.035 正是为它校准的
  co.sigma_rel_r = o.ba_rel_r;
  co.attitude_w = o.ba_att_w;
  co.huber = o.ba_huber;
  co.iters = o.ba_iters;
  co.report = false;                // 每个回环端点解一次, 打日志会淹掉一切
  std::vector<Eigen::Isometry3d> Tw;
  const auto st = ialign::optimizeSubmapCeres(Tw, T0, rel, Eigen::Vector3d::UnitZ(), ci, co);
  if (!st.ok || Tw.size() != static_cast<std::size_t>(m)) {
    for (const int c1 : mem) out.emplace_back(c1, ins_rel(c1));
    return out;
  }
  if (ba_disp) {
    std::vector<double> d;
    for (int k = 0; k < m; k++) d.push_back((Tw[k].translation() - T0[k].translation()).norm());
    std::sort(d.begin(), d.end());
    *ba_disp = d[d.size() / 2];
  }
  // 锚是中心帧, 所以 Tw[ci] 应当就是 T0[ci]; 仍显式取相对, 免得依赖这一点
  for (int k = 0; k < m; k++) out.emplace_back(mem[k], Tw[ci].inverse() * Tw[k]);
  return out;
}

static IntraCon buildLoop(const ialign::SessionData& S, const std::vector<int>& idx, int gofs,
                         const Opt& o, const ialign::CloudCovarianceEstimation& ce,
                         const char* tag) {
  IntraCon out;
  const int n = static_cast<int>(idx.size());
  const int gap = o.loop_min_gap > 0 ? o.loop_min_gap : std::max(4, o.intra_win);
  if (n < gap + 2) return out;

  // ---- 1) 位姿层面挑候选 ----
  std::vector<Eigen::Vector2d> P(n);
  std::vector<double> Y(n);
  for (int i = 0; i < n; i++) {
    const auto& T = S.frames[idx[i]].T_w_l;
    P[i] = T.translation().head<2>();
    Eigen::Vector3d f = T.linear().col(0);
    f.z() = 0;
    Y[i] = f.norm() > 1e-6 ? std::atan2(f.y(), f.x()) : 0.0;
  }
  // 累计弧长: 判"两帧之间到底走了多远"。序号差判不了 —— 等灯 45 秒有 90 多帧而车没动。
  std::vector<double> A(n, 0.0);
  for (int i = 1; i < n; i++) A[i] = A[i - 1] + (P[i] - P[i - 1]).norm();

  std::vector<std::pair<int, int>> cand;
  std::size_t n_seen = 0, n_arc = 0, n_same = 0, n_anti = 0, n_cross = 0;
  {
    std::set<std::pair<int, int>> seen;
    double acc = 1e18;
    for (int i = 0; i < n; i++) {
      if (i > 0) acc += (P[i] - P[i - 1]).norm();
      if (acc < o.loop_step) continue;   // 沿弧长限流: 不必每帧都开回环
      acc = 0;
      std::vector<std::pair<double, int>> near;
      for (int j = 0; j < n; j++) {
        if (std::abs(j - i) <= gap) continue;
        const double d = (P[j] - P[i]).norm();
        if (d > o.loop_dist) continue;
        n_seen++;
        // 基线门: 两帧之间实际走过的路程。没有基线的对(原地等灯)给不出任何信息,
        // 却会把 loop_step/loop_max 的预算吃光。
        if (o.loop_min_arc > 0 && std::abs(A[j] - A[i]) < o.loop_min_arc) continue;
        n_arc++;
        double da = std::abs(Y[j] - Y[i]);
        while (da > M_PI) da = 2 * M_PI - da;
        const double dd = da * 180.0 / M_PI;
        // **回环不判航向** —— 只按距离和里程。
        //
        // 原来有两个门(同向 dd<=loop_dyaw / 反向 dd>=180-loop_dyaw), 中间留出死区,
        // 十字路口垂直经过恰好落在里面: 实测 jjst2 在 (14,-450) 那个路口, 两次经过距离最近的
        // 一对 471<->1744 相距仅 2.08 m、航向差 127 度, 一直没建约束, 而建成的边全在路口两侧。
        // 关键帧有 INS 初值, 配准有好起点, 航向门本来只是多余的保险 ——
        // 实测 31 对死区帧对 100% 配准成功并采纳, inlier 中位 0.713 (反向边 0.791)。
        //
        // 航向差只留作**统计**打印(下面 n_same/n_anti/n_cross), 不再影响任何取舍。
        if (dd <= 30.0) n_same++;
        else if (dd >= 150.0) n_anti++;
        else n_cross++;
        near.emplace_back(d, j);
      }
      std::sort(near.begin(), near.end());
      for (int k = 0; k < std::min<int>(o.loop_per_frame, near.size()); k++) {
        const int a2 = std::min(i, near[k].second), b2 = std::max(i, near[k].second);
        if (seen.insert({a2, b2}).second) cand.emplace_back(a2, b2);
      }
    }
  }
  if (static_cast<int>(cand.size()) > o.loop_max) {
    std::sort(cand.begin(), cand.end(), [&](const auto& x, const auto& y) {
      return (P[x.first] - P[x.second]).norm() < (P[y.first] - P[y.second]).norm();
    });
    cand.resize(o.loop_max);
  }
  printf("  [%s] 回环候选筛选: 空间<%.0fm 且序号差>%d 的帧对 %zu -> 过弧长门(>=%.0fm) %zu"
         " -> 弧长抽稀后候选 %zu   [**不判航向**; 航向分布: <30度 %zu, >150度 %zu, 30~150度 %zu]\n",
         tag, o.loop_dist, gap, n_seen, o.loop_min_arc, o.loop_min_arc > 0 ? n_arc : n_seen,
         cand.size(), n_same, n_anti, n_cross);
  if (cand.empty()) return out;

  // ---- 2) 逐对配准 (并行到对, 每个线程自己加载它需要的帧) ----
  //
  // loop_submap == 0 : scan2scan, 目标端和源端各是一帧 (原行为)
  // loop_submap  > 0 : submap2submap, 每端取 +-n 帧按 **INS 相对位姿** 拼进中心帧的雷达系。
  //   反向重访时单帧的共同可见表面很少(同一根杆子被照射的是相反那一面、采样密度分布
  //   对调、遮挡相反), 拼成一段路之后共同表面大得多。
  //   **精度上限是 submap 自身的清晰度**: 它用的是还没优化的 INS 相对位姿, 所以下面
  //   要把奇偶自测重影量出来 —— 这个项目里反复验证过, target 糊到 0.1 就别指望
  //   把约束做到 0.03。
  const fs::path ldir = fs::path(o.out) / "loop";
  if (o.dump_loop > 0) fs::create_directories(ldir);
  std::atomic<int> nldump{0};

  // ---- 预计算各端点的 submap 成员帧 + 相对位姿 (按**中心帧**缓存) ----
  // loop_step 8m 而 submap 半径 25m, 所以相邻候选的端点高度重叠; 又有 loop_per_frame 条
  // 共用同一个 i。不缓存的话同一个端点的局部 BA 要重算三四遍。
  // 只缓存位姿(几十字节/帧), **不缓存点云** —— 三百个端点的点云是 GB 量级, 这个项目
  // 已经被 OOM 看守终止过四次。点云在配对时按需重读。
  std::map<int, std::vector<std::pair<int, Eigen::Isometry3d>>> smp;
  if (o.loop_submap > 0) {
    std::set<int> ctrs;
    for (const auto& cd : cand) { ctrs.insert(cd.first); ctrs.insert(cd.second); }
    std::vector<int> cv2(ctrs.begin(), ctrs.end());
    std::vector<std::vector<std::pair<int, Eigen::Isometry3d>>> res(cv2.size());
    std::vector<double> bd(cv2.size(), -1);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t u = 0; u < static_cast<std::int64_t>(cv2.size()); u++)
      res[u] = loopSubmapPoses(S, idx, cv2[u], P, o, ce, &bd[u]);
    std::vector<double> dd;
    for (std::size_t u = 0; u < cv2.size(); u++) {
      smp[cv2[u]] = res[u];
      if (bd[u] >= 0) dd.push_back(bd[u]);
    }
    std::sort(dd.begin(), dd.end());
    printf("  [%s] 回环 submap: %zu 个端点, 每端 +-%d 帧(<=%.0fm)%s",
           tag, smp.size(), o.loop_submap, o.loop_submap_radius,
           o.loop_ba ? "" : "  [--loop_ba 0: 直接用 INS 相对位姿]\n");
    if (o.loop_ba)
      printf("  局部 BA 把成员帧挪了 中位 %.3f m (%zu 个端点收敛)\n",
             dd.empty() ? -1.0 : dd[dd.size() / 2], dd.size());
  }

  std::vector<Eigen::Isometry3d> rr(cand.size());
  std::vector<char> ok(cand.size(), 0);
  std::vector<double> iv(cand.size(), -1), cv(cand.size(), -1);
  std::vector<double> nn0(cand.size(), -1), nn1(cand.size(), -1), gh(cand.size(), -1);
  std::vector<char> why(cand.size(), 0);   // 1=重叠不足 2=inlier低 3=修正过大 4=异常
  std::vector<char> is_anti(cand.size(), 0);
  Progress prog_lp(tag, "回环配准", cand.size());
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
  for (std::int64_t c = 0; c < static_cast<std::int64_t>(cand.size()); c++) {
    prog_lp.tick();
    const int i = cand[c].first, j = cand[c].second;
    // 组装一端: 中心帧 c0 的雷达系下的点。half 用于奇偶自测(只在目标端算)
    const auto assemble = [&](int c0, std::vector<Eigen::Vector4d>& pts,
                              std::vector<Eigen::Vector4d>* h0,
                              std::vector<Eigen::Vector4d>* h1) {
      // 单帧模式: 不进 smp, 直接读中心帧
      if (o.loop_submap <= 0) {
        ialign::PcdCloud pc;
        if (!loadFrame(S.frames[idx[c0]].pcd_path, o, pc)) return 0;
        pts = pc.points;
        return 1;
      }
      const auto it = smp.find(c0);
      if (it == smp.end()) return 0;
      int used = 0;
      for (const auto& [c1, Tr] : it->second) {
        ialign::PcdCloud pc;
        if (!loadFrame(S.frames[idx[c1]].pcd_path, o, pc)) continue;
        auto* hh = h0 ? ((used % 2) ? h1 : h0) : nullptr;
        for (const auto& p : pc.points) {
          pts.push_back(Tr * p);
          if (hh) hh->push_back(Tr * p);
        }
        used++;
      }
      return used;
    };
    std::vector<Eigen::Vector4d> pi, pj, hA, hB;
    const bool sm_mode = o.loop_submap > 0;
    if (assemble(i, pi, sm_mode ? &hA : nullptr, sm_mode ? &hB : nullptr) == 0) continue;
    if (assemble(j, pj, nullptr, nullptr) == 0) continue;
    if (sm_mode) {
      // 拼完必须再滤一遍: 多帧叠加后点间距远小于 frame_voxel, 不滤会让 tgt_voxel/
      // ba_pair_voxel 与点间距的比例失衡 (这个项目里 inlier 静默塌掉就是这么来的)
      std::vector<double> none;
      voxelDownsample(pi, none, o.submap_voxel);
      none.clear();
      voxelDownsample(pj, none, o.submap_voxel);
    }
    auto ci = std::make_shared<gtsam_points::PointCloudCPU>();
    auto cj = std::make_shared<gtsam_points::PointCloudCPU>();
    ci->add_points(pi);
    cj->add_points(pj);
    addCovs(ci, ce, 1);
    addCovs(cj, ce, 1);
    auto vm = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.ba_pair_voxel);
    vm->insert(*ci);
    const Eigen::Isometry3d T0 =
      S.frames[idx[i]].T_w_l.inverse() * S.frames[idx[j]].T_w_l;
    if (gtsam_points::overlap_auto(vm, cj, T0) < o.ba_min_overlap) { why[c] = 1; continue; }
    gtsam::Values vals;
    vals.insert(0, gtsam::Pose3(T0.matrix()));
    gtsam::NonlinearFactorGraph g;
    auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vm, cj);
    g.add(f);
    try {
      gtsam_points::LevenbergMarquardtExtParams lm;
      lm.setMaxIterations(o.ba_pair_iters);
      vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
    } catch (const std::exception&) {
      why[c] = 4;
      continue;
    }
    Eigen::Isometry3d T1(vals.at<gtsam::Pose3>(0).matrix());
    iv[c] = f->inlier_fraction();
    // GICP 精化: VGICP 的精度上限是体素尺度, 而回环约束的 sigma 是 0.035m 量级
    if (o.fine && iv[c] >= 0.2) {
      gtsam_points::KdTree tr(ci->points, ci->size());
      gtsam::Values v2;
      v2.insert(0, gtsam::Pose3(T1.matrix()));
      gtsam::NonlinearFactorGraph g2;
      auto ff = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(
        gtsam::Pose3(), 0, ci, cj,
        std::shared_ptr<gtsam_points::NearestNeighborSearch>(
          &tr, [](gtsam_points::NearestNeighborSearch*) {}));
      ff->set_max_correspondence_distance(o.fine_corr);
      g2.add(ff);
      try {
        gtsam_points::LevenbergMarquardtExtParams lm2;
        lm2.setMaxIterations(o.fine_iters);
        v2 = gtsam_points::LevenbergMarquardtOptimizerExt(g2, v2, lm2).optimize();
        T1 = Eigen::Isometry3d(v2.at<gtsam::Pose3>(0).matrix());
      } catch (const std::exception&) {
      }
    }
    cv[c] = (T1.translation() - T0.translation()).norm();

    // ---- 地面以上 nn: 配准前/后 ----
    // inlier_fraction 量的是**覆盖**不是精度(这个项目里已经因此判断错过一次),
    // 所以采纳判据里必须有一个真正量对齐的数。
    {
      Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
      for (const auto& p : pi) { mn = mn.cwiseMin(p.head<2>()); mx = mx.cwiseMax(p.head<2>()); }
      const auto gm = ialign::buildGroundMap({{ci.get(), Eigen::Isometry3d::Identity()}}, mn.x(),
                                            mn.y(), mx.x(), mx.y(), 8.0);
      const auto occ = buildOcc(pi, 2.0);
      gtsam_points::KdTree tr(ci->points, ci->size());
      nn0[c] = aboveGroundNN(*ci, gm, tr, *cj, T0, o.z_above, 3000, 3.0, &occ, 2.0);
      nn1[c] = aboveGroundNN(*ci, gm, tr, *cj, T1, o.z_above, 3000, 3.0, &occ, 2.0);
      // 目标端 submap 自身重影 = 这条约束的精度上限
      if (sm_mode && hA.size() > 5000 && hB.size() > 5000) {
        std::vector<double> none;
        voxelDownsample(hA, none, o.submap_voxel);
        none.clear();
        voxelDownsample(hB, none, o.submap_voxel);
        auto a0 = std::make_shared<gtsam_points::PointCloudCPU>();
        auto a1 = std::make_shared<gtsam_points::PointCloudCPU>();
        a0->add_points(hA);
        a1->add_points(hB);
        gtsam_points::KdTree t0(a0->points, a0->size());
        gh[c] = aboveGroundNN(*a0, gm, t0, *a1, Eigen::Isometry3d::Identity(), o.z_above, 5000,
                              3.0, &occ, 2.0);
      }
    }

    // 判断同向/反向, 并据此放宽反向的 inlier 门
    double da = std::abs(Y[j] - Y[i]);
    while (da > M_PI) da = 2 * M_PI - da;
    const double dd = da * 180.0 / M_PI;
    // 回环**不按航向分档**: 一律用较松的 inlier 门 + 减半的权重 (kind=3)。
    //
    // 为什么统一用保守的那一档: 有里程门(--loop_min_arc)之后, "同向重访"几乎不出现 ——
    // 实测 jjst2 的 320 条回环边里同向 0 条, 因为空间上挨着又同向的帧对全是红灯停车的
    // 零基线对, 已被里程门滤掉。剩下的反向(0.791)和交叉(0.713) inlier 水平相当,
    // 没有理由分开处理; 而用同向的 0.6 门会把交叉边拒掉一部分(实测 0.554~0.819)。
    // dd 只用于打印。
    is_anti[c] = 1;
    const double thr = o.loop_min_inlier_rev;
    // 回环的门比窗口内更严: 配错一条会把整段轨迹拽歪
    if (iv[c] < thr) { why[c] = 2; }
    else if (cv[c] > o.ba_max_corr) { why[c] = 3; }
    // nn 变差 = 配准把它推离了, 无论 inlier 多高都不要 (与 buildCross 的判据一致)
    else if (nn0[c] > 0 && nn1[c] > 0 && nn1[c] > nn0[c]) { why[c] = 5; }
    else { rr[c] = T1; ok[c] = 1; }

    // ---- debug 落盘 ----
    // 注意 --dump_cross 只管跨 session 的 scan2submap, 回环走的是这一路。
    if (o.dump_loop > 0 && nldump.load() < o.dump_loop) {
      const int seq = nldump++;
      if (seq < o.dump_loop) {
        char base[224];
        std::snprintf(base, sizeof(base), "l%04d_%s_nn%03d_g%+04d_%s_i%05d_j%05d_inl%02d_corr%03d",
                      seq, dd < 30.0 ? "same" : (dd >= 150.0 ? "anti" : "cross"),
                      static_cast<int>(std::lround(std::max(0.0, nn1[c]) * 100)),
                      static_cast<int>(std::lround((nn0[c] - nn1[c]) * 100)), ok[c] ? "OK" : "REJ",
                      i, j, static_cast<int>(std::lround(std::max(0.0, iv[c]) * 100)),
                      static_cast<int>(std::lround(std::min(9.99, std::max(0.0, cv[c])) * 100)));
        std::vector<Eigen::Vector4d> vb, va;
        vb.reserve(pj.size());
        va.reserve(pj.size());
        for (const auto& p : pj) { vb.push_back(T0 * p); va.push_back(T1 * p); }
        if (o.dump_pcd) {
          savePts(ldir / (std::string(base) + "_A.pcd"), pi);
          savePts(ldir / (std::string(base) + "_B_before.pcd"), vb);
          savePts(ldir / (std::string(base) + "_B_after.pcd"), va);
        }
        char la[160], lb[160], lm3[288];
        std::snprintf(la, sizeof(la), "TARGET %s FRAME %d (%zu PTS%s)",
                      sm_mode ? "SUBMAP AROUND" : "SCAN", i, ci->size(),
                      gh[c] > 0 ? (", SELF-GHOST " + std::to_string(gh[c]).substr(0, 5) + "M").c_str()
                                : "");
        std::snprintf(lb, sizeof(lb), "SOURCE %s FRAME %d (%zu PTS)  DYAW %.0FDEG",
                      sm_mode ? "SUBMAP AROUND" : "SCAN", j, cj->size(), dd);
        std::snprintf(lm3, sizeof(lm3),
                      "%s INLIER=%.2f CORR=%.2FM | ABOVE-GROUND NN %.3F->%.3F M | ARC GAP %.0FM",
                      ok[c] ? "ACCEPTED" : "REJECTED", std::max(0.0, iv[c]), cv[c], nn0[c], nn1[c],
                      std::abs(A[j] - A[i]));
        ialign::LoopImageParams ip;
        ip.res = 0.08;
        ip.tol = 0.15;
        ip.z_above = o.z_above;
        ialign::renderTopDownPair(*ci, *cj, T0, T1, ip, la, lb, lm3,
                                  (ldir / (std::string(base) + ".png")).string());
      }
    }
  }
  std::vector<double> inls, corrs, nnv, ghv;
  std::size_t d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 0;
  // 收集被采纳的候选用于聚类过滤: 同一地点（距离阈值内）只保留若干条
  struct AccItem { int li, lj; Eigen::Isometry3d T; double inl, corr, nn; bool anti; };
  std::vector<AccItem> accs;
  for (std::size_t c = 0; c < cand.size(); c++) {
    if (ok[c]) {
      AccItem it{cand[c].first, cand[c].second, rr[c], iv[c], cv[c], nn1[c], is_anti[c] != 0};
      accs.push_back(it);
      inls.push_back(iv[c]);
      corrs.push_back(cv[c]);
      if (nn1[c] > 0) nnv.push_back(nn1[c]);
      if (gh[c] > 0) ghv.push_back(gh[c]);
      continue;
    }
    switch (why[c]) {
      case 1: d1++; break;
      case 2: d2++; break;
      case 3: d3++; break;
      case 5: d5++; break;
      default: d4++; break;
    }
  }

  // ---- 去重: nms_loop > 0 走**四维 NMS**, 否则退回旧的中点聚类 ----
  if (o.nms_loop > 0.0 && !accs.empty()) {
    std::vector<Eigen::Vector2d> qi(accs.size()), qj(accs.size());
    std::vector<double> ql(accs.size());
    std::vector<int> qg(accs.size(), 0);   // 同一个 session, 只有一组
    for (std::size_t k = 0; k < accs.size(); k++) {
      qi[k] = P[accs[k].li];
      qj[k] = P[accs[k].lj];
      // 质量: 越小越优先。有 nn 就用 nn (它直接量对齐), 否则用 -inlier
      ql[k] = accs[k].nn > 0 ? accs[k].nn : -accs[k].inl;
    }
    const auto kp = nms4d(qi, qj, ql, qg, o.nms_loop);
    std::vector<double> arc0, arc1;
    std::size_t nk = 0;
    for (std::size_t k = 0; k < accs.size(); k++) {
      arc0.push_back(A[accs[k].li]);
      if (!kp[k]) continue;
      arc1.push_back(A[accs[k].li]);
      out.rel.emplace_back(gofs + accs[k].li, gofs + accs[k].lj, accs[k].T);
      out.kind.push_back(accs[k].anti ? 3 : 2);
      nk++;
    }
    printf("  [%s] 回环四维NMS (R=%.1fm, 两端都近才算重复): %zu -> %zu\n", tag, o.nms_loop,
           accs.size(), nk);
    reportSpread(tag, "回环 去重前", arc0);
    reportSpread(tag, "回环 去重后", arc1);
    accs.clear();   // 已经产出, 别再走下面的中点聚类
  }
  // 聚类: 把 accs 按中心点聚成簇, 每簇只保留 inl 最大的前 N 条
  const double cluster_r = o.loop_cluster_dist;
  const double cluster_r2 = cluster_r * cluster_r;
  const int max_per = std::max(1, o.loop_cluster_max);
  std::vector<int> assigned(accs.size(), -1);
  std::vector<std::vector<int>> clusters;
  for (std::size_t k = 0; k < accs.size(); ++k) {
    const Eigen::Vector2d cpos = (P[accs[k].li] + P[accs[k].lj]) * 0.5;
    int found = -1;
    for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
      const Eigen::Vector2d mpos = (P[accs[clusters[ci][0]].li] + P[accs[clusters[ci][0]].lj]) * 0.5;
      if ((mpos - cpos).squaredNorm() <= cluster_r2) { found = (int)ci; break; }
    }
    if (found < 0) { clusters.emplace_back(); found = (int)clusters.size() - 1; }
    clusters[found].push_back((int)k);
  }
  // 从每簇中按 inlier 排序, 取前 max_per 个加入 out.rel
  for (const auto &cl : clusters) {
    std::vector<int> idxs = cl;
    std::sort(idxs.begin(), idxs.end(), [&](int a, int b){ return accs[a].inl > accs[b].inl; });
    for (int t = 0; t < std::min((int)idxs.size(), max_per); ++t) {
      const AccItem &it = accs[idxs[t]];
      out.rel.emplace_back(gofs + it.li, gofs + it.lj, it.T);
      out.kind.push_back(it.anti ? 3 : 2);   // 2=同向回环 3=反向回环
    }
  }
  out.n_cand = cand.size();
  const auto med = [](std::vector<double>& v) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  };
  const auto p90 = [](std::vector<double>& v) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, v.size() * 9 / 10)];
  };
  printf("  [%s] 回环: 候选=%zu  模式=%s\n"
         "        采纳=%zu (聚类前 %zu)  剔除: 重叠不足=%zu inlier<%.2f(反向%.2f)=%zu"
         " 修正>%.1fm=%zu nn变差=%zu 异常=%zu\n"
         "        inlier 中位=%.3f  修正量 中位=%.3f p90=%.3f m  配准后 nn 中位=%.3f m\n",
         tag, cand.size(),
         o.loop_submap > 0
           ? ("submap2submap (每端 +-" + std::to_string(o.loop_submap) + " 帧)").c_str()
           : "scan2scan (单帧)",
         out.rel.size(), accs.size(), d1, o.loop_min_inlier, o.loop_min_inlier_rev, d2,
         o.ba_max_corr, d3, d5, d4, med(inls), med(corrs), p90(corrs), med(nnv));
  if (!ghv.empty())
    printf("        **目标端 submap 自身重影 中位=%.3f m** <- 这条约束的精度上限;"
           " 上面的 nn 若明显小于它, 是往模糊里塞出来的\n", med(ghv));
  if (o.dump_loop > 0)
    printf("        落盘: %s (前 %d 个候选的点云+俯视图; 文件名分数在最前, ls -r 从最差看起)\n",
           (fs::path(o.out) / "loop").string().c_str(), o.dump_loop);

  // 输出每个候选的匹配结果到 CSV, 便于离线查看反向匹配效果
  try {
    const fs::path lf = fs::path(o.out) / "loop_matches.csv";
    const bool exists = fs::exists(lf);
    std::ofstream os(lf.string(), std::ios::out | std::ios::app);
    if (os) {
      if (!exists)
        os << "tag,idx_i,idx_j,global_i,global_j,same_or_anti,arc_gap_m,dyaw_deg,inlier,corr,"
              "nn_before,nn_after,tgt_self_ghost,reason,accepted\n";
      for (std::size_t c = 0; c < cand.size(); c++) {
        const int li = cand[c].first, lj = cand[c].second;
        const int gi = gofs + li, gj = gofs + lj;
        const char* sa = is_anti[c] ? "anti" : "same";
        const double inl = iv[c];
        const double corr = cv[c];
        const int rc = static_cast<int>(why[c]);
        const int ac = ok[c] ? 1 : 0;
        double dy = std::abs(Y[lj] - Y[li]);
        while (dy > M_PI) dy = 2 * M_PI - dy;
        os << tag << ',' << li << ',' << lj << ',' << gi << ',' << gj << ',' << sa << ','
           << std::fixed << std::setprecision(2) << std::abs(A[lj] - A[li]) << ','
           << dy * 180.0 / M_PI << ',' << std::setprecision(4) << (inl < 0 ? -1.0 : inl) << ','
           << (corr < 0 ? -1.0 : corr) << ',' << nn0[c] << ',' << nn1[c] << ',' << gh[c] << ','
           << rc << ',' << ac << '\n';
      }
      os.close();
    }
  } catch (const std::exception& e) {
    (void)e;
  }
  return out;
}

// -----------------------------------------------------------------------------
// 按白化残差 rms 自校准各类约束的 sigma
//
// 原理: 残差除以假定的 sigma 之后, 若这个 sigma 就是真实离散度, 白化 rms 应当 ≈ 1。
// 实测 rms=0.13 意味着**残差比假定的小 7.7 倍**, 也就是这类约束的权重给得太轻,
// sigma 应当乘 0.13。反复解、反复更新 sigma, 就是方差分量估计 (IRLS 的一种)。
//
// 三个要点:
//   - 姿态项是**权重形式**(w 而非 sigma), 所以是 w /= rms, 方向与 sigma 相反。
//   - 每轮都从**同一个初值**重解, 而不是接着上一轮的结果继续 —— 否则 sigma 和位姿
//     一起漂, 收敛到的不是方差分量而是"越解越紧"的自证结论。
//   - **只校准测量项 (帧间 GICP、视觉), 不校准先验项 (INS 位置、姿态)**。
//     实测(WG_wuling 470,-745): 帧间 rms 0.168->1.139、视觉 0.709->1.011 都干净收敛;
//     而 INS rms 0.208->0.671 一路在爬, sigma 从 0.150 掉到 0.005 m, 姿态权重从
//     275 涨到 5356 —— 都不收敛。后果是阶段3 位移只剩 0.002m, 跨 session 约束
//     完全失效, 整个联合优化被中和。
//     原因是原理性的: 一条帧间约束牵两个位姿, 几千条约束共享 2500 个参数, 冗余度高,
//     残差 rms 近似无偏; 而 INS/姿态先验是**每个位姿一条**, 残差数 = 参数数, 冗余度≈0。
//     它的残差小不是因为 INS 准, 而是因为 INS 先验本身就是把位姿按住的那个东西 ——
//     拿它自己的残差定它自己的方差是自证。先验 sigma 必须来自外部知识(INS 绝对精度
//     0.10~0.15m)。要开这条(看它怎么发散)用 --calib_priors 1。
// -----------------------------------------------------------------------------
struct CalibIn {
  const std::vector<Eigen::Isometry3d>* T0;
  const Eigen::Isometry3d* ext0;
  const std::vector<std::tuple<int, int, Eigen::Isometry3d>>* rel;
  const std::vector<ialign::VisReprojSpecExt>* vis;
  const std::vector<int>* group;   // 每条 rel 的组号 0/1
  const std::vector<char>* fixed = nullptr;   // 哪些位姿固定 (增量建图: 前面已优化的 session)
  const std::vector<Eigen::Isometry3d>* prior = nullptr;   // INS 先验的测量值 (不给就用 T0)
  /// 与 rel 等长的 sigma **倍数** (不给 = 全 1)。--cross_per_sess 下同一次配准派生的
  /// N 条边共享同一个配准噪声, 各自 sigma 乘 sqrt(N), 否则那次测量的权重被算 N 遍。
  const std::vector<double>* sig_scale = nullptr;
};

/// @brief 解一次(可带若干轮 sigma 自校准)。返回最后一轮的位姿/外参/统计。
static ialign::SubmapCeresStats solveCalib(const CalibIn& in, ialign::SubmapCeresOpts co,
                                          int rounds, double cross_t, double cross_r,
                                          std::vector<Eigen::Isometry3d>& Tout,
                                          Eigen::Isometry3d& ext_out, const char* tag,
                                          bool calib_priors, int anchor = 0,
                                          double loop_t = 0, double loop_r = 0,
                                          double anti_w = 0.5) {
  ialign::SubmapCeresStats st;
  const std::size_t nr = in.rel->size();
  std::vector<std::array<double, 2>> rs(nr);
  const bool has_g1 = in.group && in.group->size() == nr &&
                      std::find(in.group->begin(), in.group->end(), 1) != in.group->end();
  double s0t = co.sigma_rel_t, s0r = co.sigma_rel_r, s1t = cross_t, s1r = cross_r;
  // 回环自己的 sigma; 不给就沿用窗口边的(旧行为)
  double s2t = loop_t > 0 ? loop_t : co.sigma_rel_t;
  double s2r = loop_r > 0 ? loop_r : co.sigma_rel_r;
  // 反向回环: **权重**乘 anti_w。权重 = 1/sigma^2, 所以 sigma 乘 1/sqrt(anti_w)。
  const double kanti = anti_w > 0 ? 1.0 / std::sqrt(anti_w) : 1.0;
  const bool has_loop = in.group && in.group->size() == nr &&
                        std::find_if(in.group->begin(), in.group->end(),
                                     [](int g) { return g >= 2; }) != in.group->end();
  for (int it = 0; it <= std::max(0, rounds); it++) {
    for (std::size_t c = 0; c < nr; c++) {
      const int g = (in.group && in.group->size() == nr) ? (*in.group)[c] : 0;
      if (g == 1) rs[c] = {s1t, s1r};
      else if (g == 2) rs[c] = {s2t, s2r};
      else if (g == 3) rs[c] = {s2t * kanti, s2r * kanti};
      else rs[c] = {s0t, s0r};
      // 逐条倍数 (同一次配准派生多条边时 = sqrt(N))
      if (in.sig_scale && in.sig_scale->size() == nr) {
        const double sc = (*in.sig_scale)[c] > 0 ? (*in.sig_scale)[c] : 1.0;
        rs[c][0] *= sc;
        rs[c][1] *= sc;
      }
    }
    co.report = false;
    ext_out = *in.ext0;
    st = ialign::optimizeSubmapCeresExt(Tout, ext_out, *in.T0, *in.ext0, *in.rel,
                                       Eigen::Vector3d::UnitZ(), anchor, co, *in.vis, rs,
                                       in.group ? *in.group : std::vector<int>{},
                                       in.fixed ? *in.fixed : std::vector<char>{}, in.prior);
    if (!st.ok) return st;
    printf("    [%s 轮%d] 白化rms: INS %.3f 姿态 %.3f 视觉 %s | 同session t %.3f r %.3f",
           tag, it, st.rms_ins, st.rms_att,
           st.rms_vis < 0 ? "无" : (std::to_string(st.rms_vis).substr(0, 5)).c_str(),
           st.rms_g0_t, st.rms_g0_r);
    if (has_g1) printf(" | 跨session t %.3f r %.3f", st.rms_g1_t, st.rms_g1_r);
    printf("\n");
    if (has_loop)
      printf("      回环白化rms: 同向 t %.3f r %.3f (%d 条, sigma_t=%.4f) | **反向** t %.3f"
             " r %.3f (%d 条, sigma_t=%.4f = 同向 x%.2f, 即权重 x%.2f)\n",
             st.rms_g2_t, st.rms_g2_r, st.n_rel_g2, s2t, st.rms_g3_t, st.rms_g3_r, st.n_rel_g3,
             s2t * kanti, kanti, anti_w);
    if (it == rounds) break;
    // ---- 更新 sigma ----
    const auto upd = [](double sig, double rms) {
      if (rms <= 0) return sig;
      return sig * std::min(5.0, std::max(0.2, rms));   // 单轮限幅, 防一次跳飞
    };
    const double ins_n = calib_priors ? upd(co.sigma_ins_xy, st.rms_ins) : co.sigma_ins_xy;
    const double att_n =
      (calib_priors && st.rms_att > 0)
        ? co.attitude_w / std::min(5.0, std::max(0.2, st.rms_att))
        : co.attitude_w;
    const double px_n = upd(co.sigma_px, st.rms_vis);
    const double s0t_n = upd(s0t, st.rms_g0_t), s0r_n = upd(s0r, st.rms_g0_r);
    const double s1t_n = has_g1 ? upd(s1t, st.rms_g1_t) : s1t;
    const double s1r_n = has_g1 ? upd(s1r, st.rms_g1_r) : s1r;
    // 回环那一组之前漏了 —— 只更新窗口内和跨session, 回环 sigma 一直不动, 于是它的 rms
    // 永远校不到 1。反向回环的 rms 用来更新(它才是这批数据里唯一有的那组)。
    const double rl = st.rms_g3_t > 0 ? st.rms_g3_t : st.rms_g2_t;
    const double rlr = st.rms_g3_r > 0 ? st.rms_g3_r : st.rms_g2_r;
    const double s2t_n = has_loop ? upd(s2t, rl) : s2t;
    const double s2r_n = has_loop ? upd(s2r, rlr) : s2r;
    printf("      sigma 更新: INS %.3f->%.3f%s 姿态w %.0f->%.0f 视觉 %.2f->%.2f px"
           " | 同session t %.4f->%.4f r %.5f->%.5f",
           co.sigma_ins_xy, ins_n, calib_priors ? "" : "(先验固定)", co.attitude_w, att_n,
           co.sigma_px, px_n, s0t, s0t_n, s0r, s0r_n);
    if (has_g1) printf(" | 跨session t %.4f->%.4f r %.5f->%.5f", s1t, s1t_n, s1r, s1r_n);
    if (has_loop) printf(" | 回环 t %.4f->%.4f r %.5f->%.5f", s2t, s2t_n, s2r, s2r_n);
    printf("\n");
    co.sigma_ins_xy = ins_n;
    co.attitude_w = att_n;
    co.sigma_px = px_n;
    s0t = s0t_n;
    s0r = s0r_n;
    s1t = s1t_n;
    s1r = s1r_n;
    s2t = s2t_n;
    s2r = s2r_n;
  }
  return st;
}

// =============================================================================
// 增量建图: N 个 session 依次并入
//
// 规则(用户指定): 第 1 个 session 是基准; 优化第 k 个时, 前面 k-1 个**都已经优化好且
// 固定不动**, 邻域 submap 从**前面所有 session** 的关键帧里一起拼。
//
// 位姿存档 (pose store) 就是"哪些 session 已经优化好"的那份明确清单:
//   <store>/<session_name>.csv   每帧一行 T_w_v, 头部记 base_utm / 外参 / 帧数
//   <store>/manifest.txt         人看的汇总
// 程序启动时先扫这个目录, 打印每个 session 是"已优化(跳过)"还是"待优化", 再开始干活。
// 想重做某个 session 用 --redo <名字>; 只想看清单用 --list_done。
//
// 为什么存档要记 pcd 路径而不只是下标: 这个项目已经被"下标错位"坑过一次(--load_roi
// 把帧删掉后路径表没跟着变, 边界检查还全都通过, 静默用错帧)。按路径匹配 + 要求 100%
// 覆盖, 错位就会当场报出来而不是悄悄算错。
// =============================================================================
struct PoseRec {
  std::string base_utm;                       // 基准 session 的 utm_center, 用于校验同一世界系
  Eigen::Isometry3d ext = Eigen::Isometry3d::Identity();
  std::map<std::string, Eigen::Isometry3d> by_path;   // pcd 路径 -> T_w_v
};

/// @brief 把 utm_center 格式化成字符串标签, 用于校验"存档和本次是同一个世界系"。
/// @note 只保留 3 位小数(毫米级)。位姿是"相对 utm_center 的局部坐标", center 不同就
///       不在同一系里, 拿存档的位姿直接用会整体偏移几百米而不报错 —— 所以要有这个标签。
static std::string utmTag(const Eigen::Vector2d& c) {
  char b[64];
  std::snprintf(b, sizeof(b), "%.3f_%.3f", c.x(), c.y());
  return b;
}

/// @brief 存一个 session 的位姿存档 (每帧一行 T_w_v, 头部记 base_utm 和外参)。
/// @param Tv 车体位姿, 与 S.frames 一一对应
/// @param ext 外参 T_v_l —— **必须一起存**: 边的测量都是雷达系的, 靠 T_w_l = T_w_v * ext
///            联系, 下次用 yaml 初值就和约束对不上了
/// @note 每行以 **pcd 路径**开头而不是下标 —— 按下标存, 帧集一变(--load_roi、静止块被丢)
///       就会静默错位, 这个项目被这类问题坑过一次。
static bool savePoses(const fs::path& f, const ialign::SessionData& S,
                     const std::vector<Eigen::Isometry3d>& Tv, const Eigen::Isometry3d& ext,
                     const Eigen::Vector2d& base_utm) {
  std::ofstream os(f.string());
  if (!os) return false;
  const Eigen::Quaterniond qe(ext.linear());
  os.precision(12);
  os << "# session=" << S.name << " frames=" << S.frames.size()
     << " base_utm=" << utmTag(base_utm) << "\n";
  os << "# ext_qwxyz_txyz=" << qe.w() << ' ' << qe.x() << ' ' << qe.y() << ' ' << qe.z() << ' '
     << ext.translation().x() << ' ' << ext.translation().y() << ' ' << ext.translation().z()
     << "\n";
  os << "# pcd_path,qw,qx,qy,qz,tx,ty,tz   (T_w_v)\n";
  for (std::size_t i = 0; i < S.frames.size(); i++) {
    const Eigen::Quaterniond q(Tv[i].linear());
    const auto& t = Tv[i].translation();
    os << S.frames[i].pcd_path << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z()
       << ',' << t.x() << ',' << t.y() << ',' << t.z() << '\n';
  }
  return os.good();
}

/// @brief 读位姿存档。**不做任何一致性判断** —— base_utm 对不对、帧覆盖全不全,
///        由调用方(runIncr 里的存档扫描)决定, 那里才能给出可读的原因。
/// @return false = 文件打不开或一行有效数据都没有
static bool loadPoses(const fs::path& f, PoseRec& r) {
  std::ifstream is(f.string());
  if (!is) return false;
  std::string ln;
  while (std::getline(is, ln)) {
    if (ln.empty()) continue;
    if (ln[0] == '#') {
      const auto pu = ln.find("base_utm=");
      if (pu != std::string::npos) {
        r.base_utm = ln.substr(pu + 9);
        const auto sp = r.base_utm.find(' ');
        if (sp != std::string::npos) r.base_utm = r.base_utm.substr(0, sp);
      }
      const auto pe = ln.find("ext_qwxyz_txyz=");
      if (pe != std::string::npos) {
        std::istringstream ss(ln.substr(pe + 15));
        double qw, qx, qy, qz, tx, ty, tz;
        if (ss >> qw >> qx >> qy >> qz >> tx >> ty >> tz) {
          r.ext.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
          r.ext.translation() = Eigen::Vector3d(tx, ty, tz);
        }
      }
      continue;
    }
    const auto c0 = ln.find(',');
    if (c0 == std::string::npos) continue;
    const std::string path = ln.substr(0, c0);
    std::istringstream ss(ln.substr(c0 + 1));
    std::string tok;
    double v[7];
    bool ok = true;
    for (int k = 0; k < 7; k++) {
      if (!std::getline(ss, tok, ',')) { ok = false; break; }
      v[k] = std::atof(tok.c_str());
    }
    if (!ok) continue;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized().toRotationMatrix();
    T.translation() = Eigen::Vector3d(v[4], v[5], v[6]);
    r.by_path[path] = T;
  }
  return !r.by_path.empty();
}

/// @brief 约束存档。**必须和位姿一起存**: 存档的意义是"不用重跑", 可约束只在优化时
///        才被算出来 —— 不存的话, 想画一次约束图就得把几小时的配准全部重做一遍,
///        与存档的初衷直接矛盾。
///        两端都按 pcd 路径记(与位姿存档同一个理由: 按下标记会被 ROI 变化静默错位)。
struct EdgeRec {
  std::string from, to;              // pcd 路径
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  double nn = -1;                    // 跨 session 边: 配准后的 nn; 同 session 边: -1
  int kind = 0;                      // 0=同session 1=跨session
  double ss = 1.0;                   // sigma 倍数 (--cross_per_sess); 老存档缺这列 -> 1.0
};

/// @brief 存约束存档。kind: 0=窗口内帧间 1=跨session 2=回环。
/// @note 求解时 0 和 2 同等对待(都是同 session 相对位姿), 分开记只为了统计和画图时
///       能区分"局部约束"和"回环约束"各贡献了多少。
static bool saveEdges(const fs::path& f, const std::vector<EdgeRec>& es) {
  std::ofstream os(f.string());
  if (!os) return false;
  os.precision(12);
  os << "# kind,from_pcd,to_pcd,qw,qx,qy,qz,tx,ty,tz,nn,sigma_scale"
        "   (T = 雷达系相对位姿 T_from_to)\n";
  for (const auto& e : es) {
    const Eigen::Quaterniond q(e.T.linear());
    os << e.kind << ',' << e.from << ',' << e.to << ',' << q.w() << ',' << q.x() << ',' << q.y()
       << ',' << q.z() << ',' << e.T.translation().x() << ',' << e.T.translation().y() << ','
       << e.T.translation().z() << ',' << e.nn << ',' << e.ss << '\n';
  }
  return os.good();
}

/// @brief 读约束存档, **追加**到 es (不清空), 便于多个 session 的边并到一起。
/// @return false = 文件打不开; 单行残缺会被跳过而不报错
static bool loadEdges(const fs::path& f, std::vector<EdgeRec>& es) {
  std::ifstream is(f.string());
  if (!is) return false;
  std::string ln;
  while (std::getline(is, ln)) {
    if (ln.empty() || ln[0] == '#') continue;
    std::vector<std::string> tok;
    std::size_t p = 0;
    while (p <= ln.size()) {
      const auto q = ln.find(',', p);
      tok.push_back(ln.substr(p, q == std::string::npos ? std::string::npos : q - p));
      if (q == std::string::npos) break;
      p = q + 1;
    }
    if (tok.size() < 11) continue;
    EdgeRec e;
    e.kind = std::atoi(tok[0].c_str());
    e.from = tok[1];
    e.to = tok[2];
    e.T.linear() = Eigen::Quaterniond(std::atof(tok[3].c_str()), std::atof(tok[4].c_str()),
                                      std::atof(tok[5].c_str()), std::atof(tok[6].c_str()))
                     .normalized()
                     .toRotationMatrix();
    e.T.translation() = Eigen::Vector3d(std::atof(tok[7].c_str()), std::atof(tok[8].c_str()),
                                        std::atof(tok[9].c_str()));
    e.nn = std::atof(tok[10].c_str());
    e.ss = tok.size() > 11 ? std::atof(tok[11].c_str()) : 1.0;   // 老存档没这列
    if (e.ss <= 0) e.ss = 1.0;
    es.push_back(e);
  }
  return true;
}

/// 参考帧: 来自前面所有已优化 session, 用于拼 submap 和挂跨 session 约束
struct RefFrame {
  int g = -1;                        // 全局参数下标
  int sess = 0;                      // 属于哪个 session (限流时要保证多样性)
  Eigen::Isometry3d T_w_l;           // 世界系雷达位姿
  std::string pcd_path;
};

struct CrossCon {
  std::vector<std::tuple<int, int, Eigen::Isometry3d>> rel;
  /// 与 rel 等长的 sigma **倍数**。--cross_per_sess 下同一次配准派生 N 条边, 它们共享
  /// 同一个配准噪声(不是独立测量), 所以各自的 sigma 乘 sqrt(N) —— 否则等于把那次测量
  /// 的权重算 N 遍, 就是我们在去重时要消除的那种虚高。
  std::vector<double> sig_scale;
  std::vector<double> nn0, nn1;
  int n_region = 0, n_try = 0, n_colocated = 0;
};

/// @brief 当前 session 的帧逐一配到"前面所有 session 拼出的 submap"上, 产出跨 session 约束。
static CrossCon buildCross(const std::vector<RefFrame>& ref, const ialign::SessionData& cur,
                          int gofs, const std::vector<Eigen::Isometry3d>& Tcur_l, const Opt& o,
                          const ialign::CloudCovarianceEstimation& ce) {
  CrossCon out;
  // 跨 session 候选边: 去重要等所有区域跑完再统一做 (相邻区域重叠, 分区域去重挡不住
  // 跨区域的重复), 所以这里先攒下来。
  struct CandE {
    int gi, gj;
    Eigen::Isometry3d T;
    Eigen::Vector2d pi, pj;      // 两端的世界位置 (NMS 用)
    double nn0, nn1;
    int sess;                    // 参照端属于哪个 session -> NMS 的分组
    double ss;                   // sigma 倍数 (同一次配准派生多条边时 = sqrt(N))
  };
  std::vector<CandE> cand_e;
  std::size_t n_refedge = 0;   // 目标端 BA 产出的跨session ref<->ref 约束条数
  const fs::path cdir = fs::path(o.out) / "cross";
  std::atomic<int> ndump{0};
  if (o.dump_cross > 0) fs::create_directories(cdir);
  const int nc = static_cast<int>(Tcur_l.size());
  if (ref.empty() || nc == 0) return out;

  // 每个当前帧到最近参考帧。
  // **航向门必须和距离门一起用**: 放宽距离(为了把相邻车道/并行道路纳进来)之后, 反向行驶
  // 的帧也会进来。同一位置反向经过时, 同一根杆子被照射的是相反那一面、近密远疏的采样
  // 分布对调、遮挡模式相反 —— 残差里混着配准消除不掉的系统性成分, GICP 会把它当位姿
  // 误差去"修", 修出来的是错的。
  // 取最近帧时**只在航向合格的帧里找**, 而不是先找最近的再判航向 —— 后者会因为最近那
  // 帧航向不合格就把整帧丢掉, 而旁边同向的参照帧本来是可用的。
  const auto yawOf = [](const Eigen::Isometry3d& T) {
    Eigen::Vector3d f = T.linear().col(0);
    f.z() = 0;
    return f.norm() > 1e-6 ? std::atan2(f.y(), f.x()) : 0.0;
  };
  const bool use_yaw = o.b_max_dyaw < 180.0;
  std::vector<double> ry(ref.size());
  for (std::size_t i = 0; i < ref.size(); i++) ry[i] = yawOf(ref[i].T_w_l);
  // 参照 session 的个数 (ref[i].sess 是全局 session 下标, 这里按最大值开表)
  int nsess = 1;
  for (const auto& r : ref) nsess = std::max(nsess, r.sess + 1);
  // 每个当前帧到**每个参照 session** 的最近合格帧 —— --cross_per_sess 要用它给每个
  // session 各建一条边; 关掉时仍然取全局最近的那一个 (旧行为)。
  std::vector<int> nearRS(static_cast<std::size_t>(nc) * nsess, -1);
  std::vector<double> nearDS(static_cast<std::size_t>(nc) * nsess, 1e18);
  std::vector<int> nearR(nc, -1);
  std::vector<double> nearD(nc, 1e18);
#pragma omp parallel for num_threads(o.num_threads) schedule(static)
  for (std::int64_t j = 0; j < nc; j++) {
    const Eigen::Vector2d p = Tcur_l[j].translation().head<2>();
    const double cy = yawOf(Tcur_l[j]);
    for (std::size_t i = 0; i < ref.size(); i++) {
      if (use_yaw) {
        double da = std::abs(ry[i] - cy);
        while (da > M_PI) da = 2 * M_PI - da;
        const double dd = da * 180.0 / M_PI;
        const bool same = dd <= o.b_max_dyaw;
        // 反向共位: 只影响"算不算共位/边挂谁", 不影响配准 —— submap 只按距离筛帧
        const bool anti = o.b_bidir && dd >= 180.0 - o.b_max_dyaw;
        if (!same && !anti) continue;
      }
      const double d = (ref[i].T_w_l.translation().head<2>() - p).norm();
      const std::size_t k = static_cast<std::size_t>(j) * nsess + ref[i].sess;
      if (d < nearDS[k]) { nearDS[k] = d; nearRS[k] = static_cast<int>(i); }
      if (d < nearD[j]) { nearD[j] = d; nearR[j] = static_cast<int>(i); }
    }
  }
  std::vector<int> col;
  std::size_t drop_far = 0, drop_yaw = 0;
  for (int j = 0; j < nc; j++) {
    if (nearR[j] < 0) { drop_yaw++; continue; }
    if (nearD[j] > o.b_max_dist) { drop_far++; continue; }
    col.push_back(j);
  }
  if (o.b_bidir) printf("  [--b_bidir 1] 反向共位也算 (航向差 >= %.0f 度)\n", 180.0 - o.b_max_dyaw);
  out.n_colocated = static_cast<int>(col.size());
  {
    std::vector<double> dv;
    for (int j = 0; j < nc; j++) {
      if (nearR[j] >= 0) dv.push_back(nearD[j]);
    }
    std::sort(dv.begin(), dv.end());
    const auto qq = [&](double f) {
      return dv.empty() ? -1.0
                        : dv[std::min(dv.size() - 1, static_cast<std::size_t>(f * dv.size()))];
    };
    printf("  共位帧 = %zu / %d (距离<=%.1fm 且航向差<=%.0f度; 参照系共 %zu 帧)\n"
           "    航向合格的帧到最近同向参照帧: 中位=%.2f p90=%.2f max=%.2f m"
           "   剔除: 超距离=%zu 无同向参照=%zu\n",
           col.size(), nc, o.b_max_dist, o.b_max_dyaw, ref.size(), qq(0.5), qq(0.9),
           dv.empty() ? -1.0 : dv.back(), drop_far, drop_yaw);
  }
  if (col.empty()) return out;

  double acc = 1e18;
  for (std::size_t t = 0; t < col.size(); t++) {
    if (t > 0) {
      acc += (Tcur_l[col[t]].translation().head<2>() - Tcur_l[col[t - 1]].translation().head<2>())
               .norm();
    }
    if (acc < o.region_step) continue;
    acc = 0;
    const Eigen::Vector2d ctr = Tcur_l[col[t]].translation().head<2>();
    // ---- submap: 从**前面所有 session** 的帧里取邻域 ----
    std::vector<int> ra;
    for (std::size_t i = 0; i < ref.size(); i++) {
      if ((ref[i].T_w_l.translation().head<2>() - ctr).norm() <= o.region_radius)
        ra.push_back(static_cast<int>(i));
    }
    std::vector<int> rb;
    for (const int j : col) {
      if ((Tcur_l[j].translation().head<2>() - ctr).norm() <= o.region_radius) rb.push_back(j);
    }
    if (ra.size() < 8 || rb.empty()) continue;

    // ---- 限流: 每个区域拼 submap 的参照帧数封顶 ----
    // 配准**次数**只取决于新 session 的共位帧数(与已有 session 数无关), 但每次配准的
    // **target 大小**会随 session 数线性膨胀: 一个 40m 区域现在是 96~109 帧,
    // 20 个 session 之后同一个区域有七百多帧、350 万点。次数是常数、单次在涨,
    // 总量还是线性 —— 必须在这里封住。
    // 取法: 按到区域中心的距离排序后**按 session 轮转取**, 既保证近处优先,
    // 又保证每个 session 都有代表(只按距离取会让最近的那个 session 独占名额,
    // 而多 session 的价值恰恰在于互相印证)。
    const std::size_t ra_all = ra.size();
    if (o.region_max_frames > 0 && ra.size() > static_cast<std::size_t>(o.region_max_frames)) {
      std::sort(ra.begin(), ra.end(), [&](int a2, int b2) {
        return (ref[a2].T_w_l.translation().head<2>() - ctr).norm() <
               (ref[b2].T_w_l.translation().head<2>() - ctr).norm();
      });
      std::map<int, std::vector<int>> by_sess;
      for (const int i : ra) by_sess[ref[i].sess].push_back(i);
      std::vector<int> pick;
      for (std::size_t turn = 0; pick.size() < static_cast<std::size_t>(o.region_max_frames);
           turn++) {
        bool any = false;
        for (auto& [sk, v] : by_sess) {
          (void)sk;
          if (turn >= v.size()) continue;
          any = true;
          pick.push_back(v[turn]);
          if (pick.size() >= static_cast<std::size_t>(o.region_max_frames)) break;
        }
        if (!any) break;
      }
      ra.swap(pick);
      std::sort(ra.begin(), ra.end());
    }

    // ---- 目标端: 邻域内前面所有 session 的帧**一起**做一次局部联合 BA ----
    // 目的是消掉它们在这个位置的相互不一致 (实测 0.064~0.139m), 那个不一致会被烙进
    // target, 而配准精度的上限就是 target 自身的清晰度。
    // BA 之后 A、B 处在一个**新的局部规范系**里 -> 必须把跨session 的 ref<->ref 相对位姿
    // 也发成约束, 否则 C 的边和图里的 ref 位姿不在同一个系里 (见 cross_ba_edges)。
    std::vector<Eigen::Isometry3d> Pa(ra.size());
    for (std::size_t u = 0; u < ra.size(); u++) Pa[u] = ref[ra[u]].T_w_l;
    std::vector<ialign::PcdCloud> ra_pc;
    double ba_disp = -1;
    bool ba_ok = false;
    if (o.cross_ba) {
      // 帧数封低一点: 150 帧同时带协方差+体素图约 1.2GB, 这个项目被 OOM 杀过四次
      std::vector<int> rb2 = ra;
      if (o.cross_ba_max > 0 && rb2.size() > static_cast<std::size_t>(o.cross_ba_max)) {
        std::sort(rb2.begin(), rb2.end(), [&](int x, int y) {
          return (ref[x].T_w_l.translation().head<2>() - ctr).norm() <
                 (ref[y].T_w_l.translation().head<2>() - ctr).norm();
        });
        rb2.resize(o.cross_ba_max);
        std::sort(rb2.begin(), rb2.end());
        ra.swap(rb2);
        Pa.assign(ra.size(), Eigen::Isometry3d::Identity());
        for (std::size_t u = 0; u < ra.size(); u++) Pa[u] = ref[ra[u]].T_w_l;
      }
      // 锚定离区域中心最近的那一帧 -> 局部规范系钉在它上面
      int anc = 0;
      double bd = 1e18;
      for (std::size_t u = 0; u < ra.size(); u++) {
        const double d = (Pa[u].translation().head<2>() - ctr).norm();
        if (d < bd) { bd = d; anc = static_cast<int>(u); }
      }
      std::vector<std::string> paths;
      for (const int i : ra) paths.push_back(ref[i].pcd_path);
      ba_ok = localBA(paths, Pa, anc, o, ce, &ra_pc, &ba_disp);
      if (!ba_ok) {
        for (std::size_t u = 0; u < ra.size(); u++) Pa[u] = ref[ra[u]].T_w_l;   // 失败就退回
      }
    }
    std::vector<Eigen::Vector4d> sm;
    for (std::size_t u = 0; u < ra.size(); u++) {
      const ialign::PcdCloud* pp = nullptr;
      ialign::PcdCloud tmp;
      if (u < ra_pc.size() && !ra_pc[u].points.empty()) {
        pp = &ra_pc[u];
      } else {
        if (!loadFrame(ref[ra[u]].pcd_path, o, tmp)) continue;
        pp = &tmp;
      }
      for (const auto& q : pp->points) sm.push_back(Pa[u] * q);
    }
    ra_pc.clear();
    ra_pc.shrink_to_fit();
    if (sm.size() < 20000) continue;
    // ref 全局下标 -> 在 ra/Pa 里的位置 (边要相对**BA 后**的位姿表达)
    std::map<int, std::size_t> ra_at;
    for (std::size_t u = 0; u < ra.size(); u++) ra_at[ra[u]] = u;
    // ---- 跨session 的 ref<->ref 约束: 每对 session 各一条 (取各自离中心最近的帧) ----
    // 这是"同一位置 AB/AC/BC 三条"里的 AB。**不是可选的**: 联合 BA 把 A、B 相对挪动了,
    // 不把这个挪动告诉图, 下面 C 的边就和图里的 ref 位姿不在同一个规范系里。
    if (ba_ok && o.cross_ba_edges) {
      std::map<int, std::size_t> best;
      for (std::size_t u = 0; u < ra.size(); u++) {
        const int sk = ref[ra[u]].sess;
        const auto it = best.find(sk);
        if (it == best.end() ||
            (Pa[u].translation().head<2>() - ctr).norm() <
              (Pa[it->second].translation().head<2>() - ctr).norm())
          best[sk] = u;
      }
      std::vector<std::pair<int, std::size_t>> bl(best.begin(), best.end());
      for (std::size_t x = 0; x < bl.size(); x++) {
        for (std::size_t y = x + 1; y < bl.size(); y++) {
          const std::size_t ux = bl[x].second, uy = bl[y].second;
          // 分组用**独立**编号: 这类边和 C<->ref 语义不同, 不该互相抑制
          const int gid = 1000 + std::min(bl[x].first, bl[y].first) * 10 +
                          std::max(bl[x].first, bl[y].first);
          cand_e.push_back({ref[ra[ux]].g, ref[ra[uy]].g, Pa[ux].inverse() * Pa[uy],
                            Pa[ux].translation().head<2>(), Pa[uy].translation().head<2>(),
                            -1.0, std::max(0.0, ba_disp), gid, 1.0});
          n_refedge++;
        }
      }
    }
    { std::vector<double> none; voxelDownsample(sm, none, o.submap_voxel); }
    auto tgt = std::make_shared<gtsam_points::PointCloudCPU>();
    tgt->add_points(sm);
    addCovs(tgt, ce, o.num_threads);
    auto vmm = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.tgt_voxel);
    vmm->insert(*tgt);
    gtsam_points::KdTree tree(tgt->points, tgt->size());
    Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
    for (const auto& p : sm) { mn = mn.cwiseMin(p.head<2>()); mx = mx.cwiseMax(p.head<2>()); }
    const auto gm = ialign::buildGroundMap({{tgt.get(), Eigen::Isometry3d::Identity()}}, mn.x(),
                                          mn.y(), mx.x(), mx.y(), 8.0);
    const auto occ = buildOcc(sm, 2.0);
    out.n_region++;
    const int rid = out.n_region;

    // ---- debug: 存这个区域的 target, 并量它自身有多糊 ----
    // 必须一起看: 配准精度的上限就是 target 自身的清晰度。target 糊到 0.1,
    // 就别指望把帧对到 0.03 —— 那种"更小的 nn"是往模糊里塞出来的。
    // **target 自身重影一律要量**: 配准精度的上限就是它。之前只在 --dump_cross 时才算,
    // 于是全量跑的时候这个上限完全看不见。
    double tgt_ghost = -1.0;
    {
      char nm[64];
      // submap 的落盘按**区域**封顶: 它不受 ndump 那个逐帧预算管, 全量下 130 个区域
      // x 3 份 (submap + 奇 + 偶) x 5~20MB 会写出好几 GB。
      const bool dsm = o.dump_cross > 0 && o.dump_pcd && rid <= 20;
      if (dsm) {
        std::snprintf(nm, sizeof(nm), "r%02d_submap.pcd", rid);
        savePts(cdir / nm, sm);
      }
      // 参照帧按下标奇偶拆两半, 互测 = target 自身重影
      std::vector<Eigen::Vector4d> h0, h1;
      for (std::size_t u = 0; u < ra.size(); u++) {
        ialign::PcdCloud pc2;
        if (!loadFrame(ref[ra[u]].pcd_path, o, pc2)) continue;
        auto& h = (u % 2) ? h1 : h0;
        for (const auto& p : pc2.points) h.push_back(ref[ra[u]].T_w_l * p);
      }
      std::vector<double> none;
      voxelDownsample(h0, none, o.submap_voxel);
      none.clear();
      voxelDownsample(h1, none, o.submap_voxel);
      if (h0.size() > 5000 && h1.size() > 5000) {
        auto c0 = std::make_shared<gtsam_points::PointCloudCPU>();
        auto c1 = std::make_shared<gtsam_points::PointCloudCPU>();
        c0->add_points(h0);
        c1->add_points(h1);
        gtsam_points::KdTree t0(c0->points, c0->size());
        tgt_ghost = aboveGroundNN(*c0, gm, t0, *c1, Eigen::Isometry3d::Identity(), o.z_above, 5000,
                                  3.0, &occ, 2.0);
        if (dsm) {
          std::snprintf(nm, sizeof(nm), "r%02d_submap_odd.pcd", rid);
          savePts(cdir / nm, h1);
          std::snprintf(nm, sizeof(nm), "r%02d_submap_even.pcd", rid);
          savePts(cdir / nm, h0);
        }
      }
      if (ba_ok)
        printf("    [区域%02d] 目标端联合 BA: %zu 帧, 挪动中位 %.3f m"
               "  <- 这就是这一片原本有多不一致\n", rid, ra.size(), ba_disp);
      printf("    [区域%02d] 参照帧=%zu%s 待配准=%zu  target=%zu 点  **target 自身重影=%.3f m**\n",
             rid, ra.size(),
             ra_all > ra.size() ? ("(邻域内共 " + std::to_string(ra_all) + ", 已限流)").c_str() : "",
             rb.size(), sm.size(), tgt_ghost);
    }

    // ---- 源端: 当前帧 +-cross_src_win 帧拼 submap (先局部 BA) ----
    // 单帧配到 submap 上时源侧只有一次扫描的采样, 斜交/反向时共同可见面小。
    // localBA 内部自己开 omp, 所以**必须在并行帧循环之前串行算好**, 否则嵌套并行。
    std::map<int, std::vector<std::pair<int, Eigen::Isometry3d>>> src_sm;
    if (o.cross_src_win > 0) {
      const int W = o.cross_src_win;
      double sd_sum = 0;
      int sd_n = 0;
      for (const int j : rb) {
        std::vector<int> mem;
        for (int d = -W; d <= W; d++) {
          const int j2 = j + d;
          if (j2 < 0 || j2 >= nc) continue;
          if (d != 0 && (Tcur_l[j2].translation().head<2>() -
                         Tcur_l[j].translation().head<2>()).norm() > o.loop_submap_radius)
            continue;
          mem.push_back(j2);
        }
        if (mem.size() < 3) continue;
        std::vector<std::string> paths;
        std::vector<Eigen::Isometry3d> Tm;
        int anc = 0;
        for (std::size_t u = 0; u < mem.size(); u++) {
          paths.push_back(cur.frames[mem[u]].pcd_path);
          Tm.push_back(Tcur_l[mem[u]]);
          if (mem[u] == j) anc = static_cast<int>(u);
        }
        double dsp = -1;
        localBA(paths, Tm, anc, o, ce, nullptr, &dsp);   // 失败时 Tm 不变, 照样能用
        if (dsp >= 0) { sd_sum += dsp; sd_n++; }
        std::vector<std::pair<int, Eigen::Isometry3d>> rel;
        for (std::size_t u = 0; u < mem.size(); u++)
          rel.emplace_back(mem[u], Tm[anc].inverse() * Tm[u]);   // 相对**中心帧**
        src_sm[j] = rel;
      }
      if (sd_n)
        printf("    [区域%02d] 源端 submap: 每帧 +-%d 帧, 局部 BA 挪动中位均值 %.3f m (%d 帧)\n",
               rid, W, sd_sum / sd_n, sd_n);
    }
    std::vector<Eigen::Isometry3d> res(rb.size());
    std::vector<char> ok(rb.size(), 0);
    std::vector<double> n0(rb.size(), -1), n1(rb.size(), -1);
    Progress prog_x(cur.name.c_str(), "跨session 配准 (本区域)", rb.size());
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t q = 0; q < static_cast<std::int64_t>(rb.size()); q++) {
      prog_x.tick();
      const int j = rb[q];
      // 源端: 单帧, 或 +-cross_src_win 帧拼的 submap (相对位姿来自上面的局部 BA)
      std::vector<Eigen::Vector4d> sp;
      const auto si = src_sm.find(j);
      if (si != src_sm.end()) {
        for (const auto& [j2, Tr] : si->second) {
          ialign::PcdCloud p2;
          if (!loadFrame(cur.frames[j2].pcd_path, o, p2)) continue;
          for (const auto& v : p2.points) sp.push_back(Tr * v);
        }
        // 拼完必须再滤一遍: 多帧叠加后点间距远小于 frame_voxel, 不滤会让体素图与点间距
        // 的比例失衡 (这个项目里 inlier 静默塌掉就是这么来的)
        std::vector<double> none;
        voxelDownsample(sp, none, o.submap_voxel);
      } else {
        ialign::PcdCloud pc;
        if (!loadFrame(cur.frames[j].pcd_path, o, pc)) continue;
        sp = pc.points;
      }
      if (sp.empty()) continue;
      auto src = std::make_shared<gtsam_points::PointCloudCPU>();
      src->add_points(sp);
      addCovs(src, ce, 1);
      const Eigen::Isometry3d T0 = Tcur_l[j];
      gtsam::Values vals;
      vals.insert(0, gtsam::Pose3(T0.matrix()));
      gtsam::NonlinearFactorGraph g;
      auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vmm, src);
      g.add(f);
      Eigen::Isometry3d T = T0;
      try {
        gtsam_points::LevenbergMarquardtExtParams lm;
        lm.setMaxIterations(o.iters);
        vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
        T = Eigen::Isometry3d(vals.at<gtsam::Pose3>(0).matrix());
      } catch (const std::exception&) {
        continue;
      }
      const double inl = f->inlier_fraction();
      if (o.fine && inl >= o.min_inlier) {
        gtsam::Values v2;
        v2.insert(0, gtsam::Pose3(T.matrix()));
        gtsam::NonlinearFactorGraph g2;
        auto ff = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(
          gtsam::Pose3(), 0, tgt, src,
          std::shared_ptr<gtsam_points::NearestNeighborSearch>(
            &tree, [](gtsam_points::NearestNeighborSearch*) {}));
        ff->set_max_correspondence_distance(o.fine_corr);
        g2.add(ff);
        try {
          gtsam_points::LevenbergMarquardtExtParams lm2;
          lm2.setMaxIterations(o.fine_iters);
          v2 = gtsam_points::LevenbergMarquardtOptimizerExt(g2, v2, lm2).optimize();
          T = Eigen::Isometry3d(v2.at<gtsam::Pose3>(0).matrix());
        } catch (const std::exception&) {
        }
      }
      const Eigen::Isometry3d Ta = o.clamp_planar ? clampPlanar(T, T0) : T;
      n0[q] = aboveGroundNN(*tgt, gm, tree, *src, T0, o.z_above, 3000, 3.0, &occ, 2.0);
      n1[q] = aboveGroundNN(*tgt, gm, tree, *src, Ta, o.z_above, 3000, 3.0, &occ, 2.0);
      const double corr = (Ta.translation().head<2>() - T0.translation().head<2>()).norm();
      const bool acc = inl >= o.min_inlier && corr <= o.max_corr &&
                       !(n0[q] > 0 && n1[q] > 0 && n1[q] > n0[q]);
      if (acc) {
        res[q] = Ta;
        ok[q] = 1;
      }

      // ---- debug: 这一次 scan-to-submap 的前后点云 + 俯视图 ----
      if (o.dump_cross > 0 && ndump.load() < o.dump_cross) {
        const int seq = ndump++;
        if (seq < o.dump_cross) {
          char base[224];
          std::snprintf(base, sizeof(base), "r%02d_nn%03d_g%+04d_%s_f%05d_inl%02d_corr%03d", rid,
                        static_cast<int>(std::lround(std::max(0.0, n1[q]) * 100)),
                        static_cast<int>(std::lround((n0[q] - n1[q]) * 100)), acc ? "OK" : "REJ", j,
                        static_cast<int>(std::lround(std::max(0.0, inl) * 100)),
                        static_cast<int>(std::lround(std::min(9.99, std::max(0.0, corr)) * 100)));
          std::vector<Eigen::Vector4d> vb, va;
          vb.reserve(src->size());
          va.reserve(src->size());
          for (std::size_t u = 0; u < src->size(); u++) {
            vb.push_back(T0 * src->points[u]);
            va.push_back(Ta * src->points[u]);
          }
          if (o.dump_pcd) {
            savePts(cdir / (std::string(base) + "_before.pcd"), vb);
            savePts(cdir / (std::string(base) + "_after.pcd"), va);
          }
          char la[128], lb[128], lm3[256];
          std::snprintf(la, sizeof(la), "REF SUBMAP R%02d (%zu PTS FROM %zu PREV-SESSION FRAMES, SELF-GHOST %.3FM)",
                        rid, tgt->size(), ra.size(), std::max(0.0, tgt_ghost));
          std::snprintf(lb, sizeof(lb), "%s FRAME %d", cur.name.c_str(), j);
          std::snprintf(lm3, sizeof(lm3),
                        "%s INLIER=%.2f CORR=%.2FM | ABOVE-GROUND NN %.3F->%.3F M (TGT SELF %.3F)",
                        acc ? "ACCEPTED" : "REJECTED", std::max(0.0, inl), corr, n0[q], n1[q],
                        std::max(0.0, tgt_ghost));
          ialign::LoopImageParams ip;
          ip.res = 0.08;
          ip.tol = 0.15;
          ip.z_above = o.z_above;
          ialign::renderTopDownPair(*tgt, *src, T0, Ta, ip, la, lb, lm3,
                                    (cdir / (std::string(base) + ".png")).string());
        }
      }
    }
    for (std::size_t q = 0; q < rb.size(); q++) {
      out.n_try++;
      if (!ok[q]) continue;
      // 边挂到哪些参照帧: 默认只挂全局最近的一个; --cross_per_sess 时对**每个**之前的
      // session 各挂一条 —— submap 里用了那些 session 的点, 图里却没有对应的边, 于是
      // 那两个 session 之间的一致性在图上完全没有表达 (实测 jjst5 因此漏 924 条)。
      std::vector<int> anchors;
      if (o.cross_per_sess) {
        for (int sk = 0; sk < nsess; sk++) {
          const std::size_t kk = static_cast<std::size_t>(rb[q]) * nsess + sk;
          if (nearRS[kk] >= 0 && nearDS[kk] <= o.b_max_dist) anchors.push_back(nearRS[kk]);
        }
      }
      if (anchors.empty()) anchors.push_back(nearR[rb[q]]);
      // 同一次配准派生的 N 条边共享同一个配准噪声 -> sigma 各乘 sqrt(N)
      const double ss = std::sqrt(static_cast<double>(anchors.size()));
      for (const int ai : anchors) {
        const auto& rf = ref[ai];
        // **必须用 BA 后的位姿**: target 是用 Pa 拼的, res[q] 因此在 Pa 的规范系里。
        // 用存档的 rf.T_w_l 表达这条边, 错量正好是这一片被 BA 挪动的量。
        const auto it_ra = ra_at.find(ai);
        const Eigen::Isometry3d& Pref = it_ra != ra_at.end() ? Pa[it_ra->second] : rf.T_w_l;
        // 先攒着: 去重要在**所有区域跑完之后**统一做 —— 相邻区域会重叠, 分区域各自
        // 去重挡不住跨区域的重复。
        cand_e.push_back({rf.g, gofs + rb[q], Pref.inverse() * res[q],
                          Pref.translation().head<2>(),
                          Tcur_l[rb[q]].translation().head<2>(), n0[q], n1[q], rf.sess, ss});
      }
    }
  }

  // ---- 四维 NMS: 只在**同一个参照 session** 内抑制 ----
  // 于是 "A 与 session1 的 B 建了约束后, session2 里 B 附近的 C 仍要建 A<->C" 成立 ——
  // 它们的 grp 不同。多 session 互相印证正是增量建图的价值, 不能被去重吃掉。
  // 实测(20m 格): 有跨session边的格 369->368, 连 3 个 session 的格 108->108, 没塌。
  std::vector<char> kp(cand_e.size(), 1);
  if (o.nms_cross > 0.0 && !cand_e.empty()) {
    std::vector<Eigen::Vector2d> qi, qj;
    std::vector<double> ql;
    std::vector<int> qg;
    for (const auto& c : cand_e) {
      qi.push_back(c.pi);
      qj.push_back(c.pj);
      ql.push_back(c.nn1 > 0 ? c.nn1 : 1e3);   // nn 越小越优先; 没量到的排最后
      qg.push_back(c.sess);
    }
    kp = nms4d(qi, qj, ql, qg, o.nms_cross);
    std::size_t nk = 0;
    for (const char v : kp) nk += v;
    printf("  跨session 四维NMS (R=%.1fm, 按参照 session 分组): %zu -> %zu\n", o.nms_cross,
           cand_e.size(), nk);
  }
  for (std::size_t k = 0; k < cand_e.size(); k++) {
    if (!kp[k]) continue;
    out.rel.emplace_back(cand_e[k].gi, cand_e[k].gj, cand_e[k].T);
    out.sig_scale.push_back(cand_e[k].ss);
    if (cand_e[k].nn0 > 0) out.nn0.push_back(cand_e[k].nn0);
    if (cand_e[k].nn1 > 0) out.nn1.push_back(cand_e[k].nn1);
  }
  if (n_refedge)
    printf("  目标端联合 BA 产出的跨session ref<->ref 约束: %zu 条 (同位置的 AB/AC/BC)\n",
           n_refedge);
  if (o.cross_per_sess) {
    std::map<int, int> per;
    for (std::size_t k = 0; k < cand_e.size(); k++) {
      if (kp[k]) per[cand_e[k].sess]++;
    }
    printf("  --cross_per_sess: 按参照 session 分开的边数:");
    for (const auto& [sk, n] : per) printf(" s%d=%d", sk, n);
    printf("\n");
  }
  return out;
}

// -----------------------------------------------------------------------------
// 约束图: 画出来 + 写 g2o
//
// 画的是**位姿图本身**, 不是点云: 每个 session 的关键帧位姿用一种颜色的点, 跨 session
// 的 scan-to-submap 约束用线连起来 (线的两端 = 参照帧 和 被并入的帧)。
//
// 为什么值得单独画: 跨 session 约束是整个增量建图里唯一把两趟数据绑在一起的东西, 而它
// 的分布**极不均匀** —— 只在"共位且能配上"的地方才有。看数字("建成 741 条")完全看不出
// 这 741 条是均匀铺在全程, 还是挤在两三个路口。后者意味着中间那些没约束的路段全靠
// 各自的 BA 撑着, 一致性没有任何保证 —— 这正是要用眼睛确认的事。
// -----------------------------------------------------------------------------
struct GraphEdge {
  int gi = -1, gj = -1;     // 全局位姿下标
  int si = -1, sj = -1;     // 两端各属于哪个 session
  Eigen::Isometry3d T;      // 测量 (雷达系相对位姿)
  double nn = -1;           // 配准后的 nn (质量), <0 = 未知
  double ss = 1.0;          // sigma 倍数 (--cross_per_sess 下 = sqrt(同一次配准派生的边数))
};

/// 8 种区分度较高的颜色 (session 用)
static const std::uint8_t kSessColor[8][3] = {
  {230, 60, 60},   {60, 170, 230},  {90, 220, 90},   {235, 190, 50},
  {200, 100, 230}, {255, 140, 40},  {80, 230, 210},  {230, 120, 160},
};

/// @brief 在 RGB 缓冲上画一条**带透明度**的线 (整数插值, 无抗锯齿)。
/// @param alpha 0~1; 约束图里用它表达配准质量, 所以必须支持叠加而不是直接覆盖
/// @note 越界的像素直接跳过, 不做裁剪计算。
static void drawLineRGB(std::vector<std::uint8_t>& img, int W, int H, int x0, int y0, int x1,
                       int y1, const std::uint8_t* c, double alpha) {
  const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  const int n = std::max(1, std::max(dx, dy));
  for (int t = 0; t <= n; t++) {
    const int x = x0 + (x1 - x0) * t / n, y = y0 + (y1 - y0) * t / n;
    if (x < 0 || y < 0 || x >= W || y >= H) continue;
    std::uint8_t* p = &img[(static_cast<std::size_t>(y) * W + x) * 3];
    for (int k = 0; k < 3; k++)
      p[k] = static_cast<std::uint8_t>(std::min(255.0, p[k] * (1 - alpha) + c[k] * alpha));
  }
}

/// @brief 在 RGB 缓冲上画一个实心圆点 (直接覆盖, 不混色)。
/// @param r 半径(像素); r=1 就是 5 个像素的十字块
static void drawDot(std::vector<std::uint8_t>& img, int W, int H, int x, int y, int r,
                   const std::uint8_t* c) {
  for (int dy = -r; dy <= r; dy++) {
    for (int dx = -r; dx <= r; dx++) {
      if (dx * dx + dy * dy > r * r) continue;
      const int xx = x + dx, yy = y + dy;
      if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
      std::uint8_t* p = &img[(static_cast<std::size_t>(yy) * W + xx) * 3];
      p[0] = c[0];
      p[1] = c[1];
      p[2] = c[2];
    }
  }
}

/// @brief 画约束图。poses 是**世界系雷达位姿**, sess_of[g] 给出每个位姿属于哪个 session。
static void drawConstraintGraph(const std::vector<Eigen::Isometry3d>& poses,
                               const std::vector<int>& sess_of,
                               const std::vector<std::string>& sess_names,
                               const std::vector<GraphEdge>& cross, const fs::path& out,
                               int px_long = 1800, const Eigen::Vector3d* zoom = nullptr) {
  if (poses.empty()) return;
  Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
  if (zoom) {
    // 放大视图: 约束连的是"最近的参照帧", 两端本来就只差几米 —— 全局图上那是二十来个
    // 像素, 看不出结构。必须另出一张放大的才能看清约束到底怎么连的。
    mn = Eigen::Vector2d(zoom->x() - zoom->z() * 0.5, zoom->y() - zoom->z() * 0.5);
    mx = Eigen::Vector2d(zoom->x() + zoom->z() * 0.5, zoom->y() + zoom->z() * 0.5);
  } else {
    for (const auto& T : poses) {
      mn = mn.cwiseMin(T.translation().head<2>());
      mx = mx.cwiseMax(T.translation().head<2>());
    }
  }
  if (!zoom) {
    const double pad = 30.0;
    mn.array() -= pad;
    mx.array() += pad;
  }
  const double span = std::max(mx.x() - mn.x(), mx.y() - mn.y());
  const double sc = px_long / std::max(1.0, span);
  const int W = static_cast<int>((mx.x() - mn.x()) * sc) + 1;
  const int H = static_cast<int>((mx.y() - mn.y()) * sc) + 1 + 90;   // 底部留图例
  std::vector<std::uint8_t> img(static_cast<std::size_t>(W) * H * 3, 18);
  const auto px = [&](const Eigen::Vector3d& t) {
    return std::pair<int, int>(static_cast<int>((t.x() - mn.x()) * sc),
                               static_cast<int>((mx.y() - t.y()) * sc));
  };

  // ---- 跨 session 约束: 先画线, 让位姿点压在上面 ----
  // 线的颜色按**被并入的那一侧**的 session 走, 这样一眼能看出"谁被绑到了谁"。
  for (const auto& e : cross) {
    if (e.gi < 0 || e.gj < 0 || e.gi >= static_cast<int>(poses.size()) ||
        e.gj >= static_cast<int>(poses.size()))
      continue;
    const auto [x0, y0] = px(poses[e.gi].translation());
    const auto [x1, y1] = px(poses[e.gj].translation());
    const std::uint8_t* c = kSessColor[e.sj % 8];
    // 质量好的线更实: nn 小 = 配得准
    const double a = e.nn > 0 ? std::max(0.25, std::min(0.95, 0.12 / e.nn)) : 0.6;
    const int lw = zoom ? 2 : 1;   // 加粗, 否则几米长的边在全局图上根本看不见
    for (int d = -lw; d <= lw; d++) {
      drawLineRGB(img, W, H, x0, y0 + d, x1, y1 + d, c, a);
      drawLineRGB(img, W, H, x0 + d, y0, x1 + d, y1, c, a);
    }
  }
  // ---- 位姿点 ----
  // 有跨 session 约束的帧画大点 —— "约束在哪儿"比"约束有多少条"重要得多:
  // 741 条边可以是铺满全程, 也可以是挤在两三个路口, 后者意味着中间路段没有任何约束。
  std::vector<char> constrained(poses.size(), 0);
  for (const auto& e : cross) {
    if (e.gi >= 0 && e.gi < static_cast<int>(poses.size())) constrained[e.gi] = 1;
    if (e.gj >= 0 && e.gj < static_cast<int>(poses.size())) constrained[e.gj] = 1;
  }
  const int rbase = zoom ? 2 : 1, rcon = zoom ? 5 : 3;
  for (std::size_t g = 0; g < poses.size(); g++) {
    const auto [x, y] = px(poses[g].translation());
    if (constrained[g]) {
      const std::uint8_t wht[3] = {255, 255, 255};
      drawDot(img, W, H, x, y, rcon, wht);
      drawDot(img, W, H, x, y, rcon - 1, kSessColor[sess_of[g] % 8]);
    } else {
      drawDot(img, W, H, x, y, rbase, kSessColor[sess_of[g] % 8]);
    }
  }
  // ---- 图例 ----
  int ty = H - 84;
  ialign::drawText(img, W, H, 8, ty, "CONSTRAINT GRAPH  DOTS=KEYFRAME POSES  LINES=CROSS-SESSION SCAN2SUBMAP", 210, 210, 210, 2);
  ty += 22;
  int tx = 8;
  for (std::size_t k = 0; k < sess_names.size(); k++) {
    const std::uint8_t* c = kSessColor[k % 8];
    drawDot(img, W, H, tx + 6, ty + 7, 5, c);
    std::string nm = sess_names[k];
    if (nm.size() > 28) nm = nm.substr(nm.size() - 28);
    for (auto& ch : nm) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    tx += 18 + ialign::drawText(img, W, H, tx + 18, ty, nm, c[0], c[1], c[2], 2) + 26;
  }
  ty += 24;
  char b[200];
  std::size_t ncon = 0;
  for (const char c : constrained) ncon += c;
  std::snprintf(b, sizeof(b),
                "%zu POSES  %zu CROSS EDGES  %zu POSES CONSTRAINED (BIG WHITE-RIMMED DOTS)  OPACITY~QUALITY",
                poses.size(), cross.size(), ncon);
  ialign::drawText(img, W, H, 8, ty, b, 170, 170, 170, 2);

  ialign::writePng(out.string(), W, H, img);
  printf("  约束图: %s  (%dx%d px, %.2f m/px)\n", out.string().c_str(), W, H, 1.0 / sc);
}

/// @brief 把位姿图写成 g2o。顶点是 T_w_v (车体位姿), 边是雷达系相对位姿。
///
/// 注意单位/约定: g2o 的 EDGE_SE3:QUAT 测量是 **T_i_j**, 与这里的 rel 一致。
/// 信息矩阵按各类约束自己的 sigma 给 (平移 1/sigma_t^2, 旋转 1/sigma_r^2), 只填上三角。
/// 已优化并固定的 session 的顶点写 FIX —— 否则别人拿这个文件去跑会得到完全不同的解,
/// 因为增量建图里那些顶点本来就是不动的参照。
static bool saveG2o(const fs::path& f, const std::vector<Eigen::Isometry3d>& Tv,
                   const std::vector<int>& sess_of, const std::vector<char>& fixed,
                   const std::vector<std::tuple<int, int, Eigen::Isometry3d>>& intra,
                   double s_it, double s_ir, const std::vector<GraphEdge>& cross, double s_ct,
                   double s_cr, const std::vector<Eigen::Isometry3d>& Tins, double s_ins,
                   double ins_z_scale, double att_w, int anchor,
                   const std::vector<std::string>& sess_names) {
  std::ofstream os(f.string());
  if (!os) return false;
  os.precision(10);
  const auto vtx = [&](int g, const Eigen::Isometry3d& T) {
    const Eigen::Quaterniond q(T.linear());
    os << "VERTEX_SE3:QUAT " << g << ' ' << T.translation().x() << ' ' << T.translation().y() << ' '
       << T.translation().z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
       << '\n';
  };
  const auto edge = [&](int i, int j, const Eigen::Isometry3d& T, double st, double sr) {
    const Eigen::Quaterniond q(T.linear());
    os << "EDGE_SE3:QUAT " << i << ' ' << j << ' ' << T.translation().x() << ' '
       << T.translation().y() << ' ' << T.translation().z() << ' ' << q.x() << ' ' << q.y() << ' '
       << q.z() << ' ' << q.w();
    const double it = 1.0 / (st * st), ir = 1.0 / (sr * sr);
    const double inf[6] = {it, it, it, ir, ir, ir};
    for (int a = 0; a < 6; a++) {
      for (int b = a; b < 6; b++) os << ' ' << (a == b ? inf[a] : 0.0);
    }
    os << '\n';
  };
  // INS/姿态先验也必须写进去 —— 它们是求解时的主要约束(每个位姿一条)。只写相对边的话
  // 这个图缺少绝对基准, 别人拿去跑会整体漂走, 得到的解和这里的结果没关系。
  // 用 g2o 的 EDGE_SE3_PRIOR (需要一个 SE3 offset 参数块)。
  const auto prior = [&](int i, const Eigen::Isometry3d& T) {
    const Eigen::Quaterniond q(T.linear());
    os << "EDGE_SE3_PRIOR " << i << " 0 " << T.translation().x() << ' ' << T.translation().y()
       << ' ' << T.translation().z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
       << q.w();
    // 平移: 1/sigma^2 (z 方向按 ins_z_scale 缩); 旋转: roll/pitch 用姿态权重, yaw 给 0
    // —— 求解器里的姿态先验是"重力方向分解后只约束 roll/pitch, yaw 自由"。
    // 这里把 yaw 的信息置 0 是那个约束在**对角信息矩阵**下最接近的表达: 车体近水平时
    // 世界 z 与车体 z 基本重合, 所以 yaw 那一维就是世界 yaw。不完全等价, 已在此说明。
    const double it = 1.0 / (s_ins * s_ins);
    const double itz = it * ins_z_scale * ins_z_scale;
    const double ir = att_w * att_w;
    const double inf[6] = {it, it, itz, ir, ir, 0.0};
    for (int a = 0; a < 6; a++) {
      for (int b = a; b < 6; b++) os << ' ' << (a == b ? inf[a] : 0.0);
    }
    os << '\n';
  };

  os << "# 增量建图位姿图 —— **全部 session 的位姿和全部约束**。\n";
  os << "# 顶点 = **优化后**的位姿估计; 边 = 测量值(测量不随优化改变, 这是 g2o 的约定)。\n";
  os << "#   所以直接跑 g2o 优化这个文件, 结果应当基本不动 —— 它已经是收敛点了。\n";
  os << "#   INS 先验边的测量是**优化前**的 INS 位置, 那才是这条约束的观测量。\n";
  os << "# 顶点 VERTEX_SE3:QUAT      = T_w_v (车体位姿, 优化后)\n";
  os << "# 边   EDGE_SE3:QUAT        = 雷达系相对位姿 T_i_j (同session GICP / 跨session scan2submap)\n";
  os << "# 边   EDGE_SE3_PRIOR       = INS 位置 + 重力姿态先验 (每个位姿一条)\n";
  os << "# 同session边 sigma: t=" << s_it << " r=" << s_ir << " | 跨session边 sigma: t=" << s_ct
     << " r=" << s_cr << " | INS sigma=" << s_ins << " (z x" << ins_z_scale
     << ") | 姿态权重=" << att_w << "\n";
  os << "# 只 FIX 锚定帧(定 6 个规范自由度)。导出的是**完整联合问题**: 每个顶点都有先验边,\n";
  os << "#   所有 session 的位姿都可动 —— 增量求解时把前面的 session 钉死是那一步的策略,\n";
  os << "#   不是这张图的性质。增量求解中被钉过的 session:";
  {
    bool any = false;
    for (std::size_t k = 0; k < sess_names.size(); k++) {
      bool all_fixed = true, has = false;
      for (std::size_t g = 0; g < Tv.size() && g < sess_of.size(); g++) {
        if (sess_of[g] != static_cast<int>(k)) continue;
        has = true;
        if (g >= fixed.size() || !fixed[g]) all_fixed = false;
      }
      if (has && all_fixed) { os << ' ' << sess_names[k]; any = true; }
    }
    os << (any ? "\n" : " (无)\n");
  }
  os << "# session 分段:\n";
  {
    std::vector<int> first(sess_names.size(), -1), last(sess_names.size(), -1);
    for (std::size_t g = 0; g < Tv.size(); g++) {
      const int k = g < sess_of.size() ? sess_of[g] : 0;
      if (first[k] < 0) first[k] = static_cast<int>(g);
      last[k] = static_cast<int>(g);
    }
    for (std::size_t k = 0; k < sess_names.size(); k++)
      os << "#   [" << k << "] " << sess_names[k] << "  顶点 " << first[k] << ".." << last[k]
         << "\n";
  }
  os << "# 顶点 id -> pcd 路径的对应表在同名的 _index.csv 里 —— 标准 g2o 格式没有地方放它,\n";
  os << "#   而重新构造 submap 必须知道去哪儿读点云。别把路径塞进注释: 注释经别的工具\n";
  os << "#   round-trip 之后就没了。\n";
  os << "PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1\n";
  for (std::size_t g = 0; g < Tv.size(); g++) vtx(static_cast<int>(g), Tv[g]);
  for (const auto& [i, j, T] : intra) edge(i, j, T, s_it, s_ir);
  for (const auto& e : cross) edge(e.gi, e.gj, e.T, s_ct, s_cr);
  // **每个顶点都写先验**, 且**只钉锚定帧**。
  // 这里导出的是"完整的联合问题", 不是增量求解时的固定关系 —— 增量求解把前面的 session
  // 钉死是那一步的策略, 但如果照搬到文件里, 已优化过的 session 会全部 FIX、连先验都不写,
  // 别人拿到的就是一张钉死的图, 没法优化也看不出约束强弱(实测第一版就是 FIX 550/先验 0)。
  // 哪些 session 在增量求解时被钉过, 写进注释里, 信息不丢。
  std::size_t n_prior = 0;
  for (std::size_t g = 0; g < Tv.size() && g < Tins.size(); g++) {
    prior(static_cast<int>(g), Tins[g]);
    n_prior++;
  }
  std::size_t n_fix = 0;
  if (anchor >= 0 && anchor < static_cast<int>(Tv.size())) {
    os << "FIX " << anchor << '\n';
    n_fix = 1;
  }

  // 按 session 报一遍, 缺边的当场看得见
  std::vector<std::size_t> ei(sess_names.size(), 0), ec(sess_names.size(), 0),
    nv(sess_names.size(), 0);
  for (std::size_t g = 0; g < Tv.size(); g++) nv[g < sess_of.size() ? sess_of[g] : 0]++;
  for (const auto& [i, j, T] : intra) {
    (void)T;
    if (i >= 0 && i < static_cast<int>(sess_of.size())) ei[sess_of[i]]++;
  }
  for (const auto& e : cross) {
    if (e.sj >= 0 && e.sj < static_cast<int>(ec.size())) ec[e.sj]++;
  }
  printf("  g2o: %s\n    顶点 %zu | 同session边 %zu | 跨session边 %zu | 先验边 %zu | FIX %zu\n",
         f.string().c_str(), Tv.size(), intra.size(), cross.size(), n_prior, n_fix);
  for (std::size_t k = 0; k < sess_names.size(); k++) {
    printf("      [%zu] %-34s 顶点 %-5zu 同session边 %-6zu 跨session边 %zu%s\n", k,
           sess_names[k].c_str(), nv[k], ei[k], ec[k],
           ei[k] == 0 ? "   **一条帧间边都没有 —— 这个 session 的约束没进来!**" : "");
  }
  return os.good();
}

/// @brief g2o 的顶点 id -> pcd 路径索引。**必须和 g2o 一起写**, 见 saveG2o 里的说明。
static bool saveG2oIndex(const fs::path& f, const std::vector<int>& sess_of,
                        const std::vector<std::string>& sess_names,
                        const std::vector<std::string>& paths,
                        const Eigen::Isometry3d& ext) {
  std::ofstream os(f.string());
  if (!os) return false;
  os.precision(12);
  // **外参必须跟着图走**。顶点存的是车体位姿 T_w_v, 而所有边的测量都是**雷达系**的相对
  // 位姿 —— 两者靠 T_w_l = T_w_v * T_v_l 联系。下一轮如果 T_v_l 换成了 yaml 初值,
  // 位姿和约束就对不上, 联合优化只能靠扭曲位姿去凑。
  // (实测: 漏掉这一条时, 老 session 在联合优化里被挪了 3.56m, 而正常只有 0.05m。)
  {
    const Eigen::Quaterniond q(ext.linear());
    os << "# ext_qwxyz_txyz=" << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
       << ext.translation().x() << ' ' << ext.translation().y() << ' ' << ext.translation().z()
       << "\n";
  }
  os << "vertex_id,session,pcd_path\n";
  for (std::size_t g = 0; g < paths.size(); g++)
    os << g << ',' << sess_names[g < sess_of.size() ? sess_of[g] : 0] << ',' << paths[g] << '\n';
  return os.good();
}

/// 从 g2o + 索引里读回来的东西
struct LoadedGraph {
  std::map<std::string, Eigen::Isometry3d> pose_of;                    // pcd 路径 -> T_w_v
  std::vector<std::tuple<std::string, std::string, Eigen::Isometry3d>> edges;  // 两端按路径
  std::set<std::string> sessions;                                      // 覆盖了哪些 session
  Eigen::Isometry3d ext = Eigen::Isometry3d::Identity();
  bool has_ext = false;                                                // 图里带了外参
};

/// @brief 读 g2o + 同名 _index.csv。顶点/边一律换算成 **pcd 路径**, 不用整数 id ——
///        新一轮的 session 顺序、帧集(比如 --load_roi 或静止块被丢掉)都可能变,
///        照搬整数 id 会静默错位, 这个项目已经被这类问题坑过一次。
static bool loadG2o(const fs::path& f, LoadedGraph& out) {
  fs::path idx = f;
  idx.replace_extension();
  idx += "_index.csv";
  if (!fs::exists(f) || !fs::exists(idx)) {
    printf("  !! --load_g2o: 找不到 %s 或它的索引 %s\n", f.string().c_str(), idx.string().c_str());
    return false;
  }
  std::map<int, std::string> path_of;
  {
    std::ifstream is(idx.string());
    std::string ln;
    while (std::getline(is, ln)) {
      if (ln.empty() || ln[0] == 'v') continue;
      if (ln[0] == '#') {
        const auto pe = ln.find("ext_qwxyz_txyz=");
        if (pe != std::string::npos) {
          std::istringstream ss(ln.substr(pe + 15));
          double qw, qx, qy, qz, tx, ty, tz;
          if (ss >> qw >> qx >> qy >> qz >> tx >> ty >> tz) {
            out.ext.linear() =
              Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
            out.ext.translation() = Eigen::Vector3d(tx, ty, tz);
            out.has_ext = true;
          }
        }
        continue;
      }
      const auto c1 = ln.find(','), c2 = ln.find(',', c1 + 1);
      if (c1 == std::string::npos || c2 == std::string::npos) continue;
      path_of[std::atoi(ln.substr(0, c1).c_str())] = ln.substr(c2 + 1);
      out.sessions.insert(ln.substr(c1 + 1, c2 - c1 - 1));
    }
  }
  std::ifstream is(f.string());
  std::string tok;
  std::size_t n_v = 0, n_e = 0, n_miss = 0;
  while (is >> tok) {
    if (tok == "VERTEX_SE3:QUAT") {
      int id;
      double x, y, z, qx, qy, qz, qw;
      is >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
      const auto it = path_of.find(id);
      if (it == path_of.end()) { n_miss++; continue; }
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
      T.translation() = Eigen::Vector3d(x, y, z);
      out.pose_of[it->second] = T;
      n_v++;
    } else if (tok == "EDGE_SE3:QUAT") {
      int i, j;
      double x, y, z, qx, qy, qz, qw;
      is >> i >> j >> x >> y >> z >> qx >> qy >> qz >> qw;
      for (int k = 0; k < 21; k++) { double d; is >> d; }
      const auto a1 = path_of.find(i), b1 = path_of.find(j);
      if (a1 == path_of.end() || b1 == path_of.end()) { n_miss++; continue; }
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
      T.translation() = Eigen::Vector3d(x, y, z);
      out.edges.emplace_back(a1->second, b1->second, T);
      n_e++;
    } else {
      std::getline(is, tok);   // 其余(先验/FIX/注释/参数)跳过 —— 先验按 INS 重新生成
    }
  }
  if (!out.has_ext)
    printf("  !! 图里没有外参 —— 这是老版本写的文件。位姿和雷达系约束会对不上,\n"
           "     联合优化只能靠扭曲位姿去凑(实测能把老 session 挪 3.5m)。请重新生成一次。\n");
  printf("  [load_g2o] %s\n    顶点 %zu, 边 %zu, 覆盖 session %zu 个%s\n", f.string().c_str(),
         n_v, n_e, out.sessions.size(),
         n_miss ? ("  (有 " + std::to_string(n_miss) + " 条在索引里找不到, 已跳过)").c_str() : "");
  return n_v > 0;
}

/// @brief 丢掉与主轨迹**断开**的帧。判据是**连通域**, 不是"半径内有没有别的帧"。
///
/// 为什么必须用连通域: 实测 WG_wuling 的 jjst2 里, clip _000000 的 54 帧全部停在起点
/// (878.5,-692.6) 一动不动 —— 彼此间距 0.02m —— 然后 #53 到 #54 直接跳了 118.9m
/// (中间的 clip _000001 不在数据里, 那段行驶过程缺失)。这 54 帧离主图 85m,
/// 在导出的点云里就是明显脱离主路线的一块。
/// 而"半径内没有别的帧"这个判据对它**原理上无效**: 它们互相挨着, 每一帧的最近邻都是 0m。
/// 一帧孤立是连通域大小为 1 的特例, 所以连通域判据把两种情况一起覆盖了。
///
/// 规则(实测定出来的, 不是拍的): 把相距 <= radius 的帧连成连通域, 然后**丢掉自身空间
/// 尺度 < static_ext 的域** —— 也就是"这一块里车根本没动过"。
///
/// 为什么不是"只保留最大连通域": 实测三个 session 一共 20 个连通域, 除了上面那一块,
/// 其余的自身尺度都是 175~624m —— 全是**真实路段**, 只是 clip 之间缺数据(这批 clip 是
/// 抽出来的片段, 彼此隔着 100~800m)。jjst3 裂成 10 块、最大的只占 21%, 按"保留最大"
/// 会丢掉 79% 的真实数据。整个数据集里尺度为 0 的**只有一块**, 正是要丢的那块。
///
/// 停在路口等红灯的那种帧不会被误伤: 它们和前后行驶段连通, 属于尺度几百米的大域。
/// @param do_drop false = 只统计不丢帧 (--drop_frames 0)。分析照做、"本该丢哪些"照打印,
///                但所有帧都留下参与优化 —— 判读需要的信息一条不少, 只是不动数据。
static void dropIsolated(std::vector<ialign::SessionData*> ss, double radius, double static_ext,
                        bool do_drop = true) {
  printf("\n=== 断连/静止块检查 (相距 <=%.1fm 视为相连; 自身尺度 <%.1fm 的块%s) ===\n",
         radius, static_ext, do_drop ? "丢掉" : "**本该丢但 --drop_frames 0, 全部保留**");
  for (auto* S : ss) {
    const int n = static_cast<int>(S->frames.size());
    if (n < 2) continue;
    std::vector<double> d(n, 1e18);
#pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
      const Eigen::Vector2d p = S->frames[i].T_w_l.translation().head<2>();
      for (int j = 0; j < n; j++) {
        if (j == i) continue;
        const double dd = (S->frames[j].T_w_l.translation().head<2>() - p).norm();
        if (dd < d[i]) d[i] = dd;
      }
    }
    std::vector<double> sd = d;
    std::sort(sd.begin(), sd.end());
    const auto q = [&](double f) {
      return sd[std::min(sd.size() - 1, static_cast<std::size_t>(f * sd.size()))];
    };
    printf("  %-34s 帧=%-5d 到最近同session帧: 中位=%.2f p90=%.2f p99=%.2f max=%.2f m\n",
           S->name.c_str(), n, q(0.5), q(0.9), q(0.99), sd.back());
    // 最孤立的几帧: 连坐标和路径一起报, 才能直接去数据里核对
    {
      std::vector<int> ord(n);
      for (int i = 0; i < n; i++) ord[i] = i;
      std::partial_sort(ord.begin(), ord.begin() + std::min(n, 5), ord.end(),
                        [&](int a1, int b1) { return d[a1] > d[b1]; });
      printf("    最孤立的 %d 帧:\n", std::min(n, 5));
      for (int u = 0; u < std::min(n, 5); u++) {
        const auto& t = S->frames[ord[u]].T_w_l.translation();
        printf("      #%-5d 最近邻=%7.2f m  位置=(%8.1f, %8.1f, %6.1f)  %s\n", ord[u], d[ord[u]],
               t.x(), t.y(), t.z(), S->frames[ord[u]].pcd_path.c_str());
      }
    }
    if (radius <= 0) continue;

    // ---- 连通域 (并查集; 相距 <= radius 就并到一起) ----
    std::vector<int> par(n);
    for (int i = 0; i < n; i++) par[i] = i;
    std::function<int(int)> find = [&](int x) {
      while (par[x] != x) { par[x] = par[par[x]]; x = par[x]; }
      return x;
    };
    // 先按 radius 大小分格, 只比同格和邻格 —— 全对全在 2500 帧上是 600 万次, 还能忍,
    // 但帧数再大就不行了。
    // (a) **同一 clip 内时序相邻的帧无条件相连** —— 那是一段连续录制, 车在开,
    //     间距大只说明车快, 不代表数据断开。少了这一条, 10m 半径会把正常轨迹
    //     在每个稍快的路段切断(实测 jjst2 被切成 5 块, 最大的只剩 55%)。
    //     真正的断开发生在 **clip 之间** —— 比如 jjst2 缺了 clip _000001。
    const auto clip_of = [](const std::string& p2) {
      return fs::path(p2).parent_path().parent_path().parent_path().filename().string();
    };
    for (int i = 1; i < n; i++) {
      if (clip_of(S->frames[i].pcd_path) != clip_of(S->frames[i - 1].pcd_path)) continue;
      const int a2 = find(i), b2 = find(i - 1);
      if (a2 != b2) par[a2] = b2;
    }
    std::map<std::pair<long, long>, std::vector<int>> grid;
    for (int i = 0; i < n; i++) {
      const auto& t = S->frames[i].T_w_l.translation();
      grid[{static_cast<long>(std::floor(t.x() / radius)),
            static_cast<long>(std::floor(t.y() / radius))}].push_back(i);
    }
    for (const auto& [c, v] : grid) {
      for (long dx = -1; dx <= 1; dx++) {
        for (long dy = -1; dy <= 1; dy++) {
          const auto it = grid.find({c.first + dx, c.second + dy});
          if (it == grid.end()) continue;
          for (const int i : v) {
            for (const int j : it->second) {
              if (i >= j) continue;
              if ((S->frames[i].T_w_l.translation().head<2>() -
                   S->frames[j].T_w_l.translation().head<2>()).norm() > radius)
                continue;
              const int a2 = find(i), b2 = find(j);
              if (a2 != b2) par[a2] = b2;
            }
          }
        }
      }
    }
    std::map<int, std::vector<int>> comp;
    for (int i = 0; i < n; i++) comp[find(i)].push_back(i);
    if (comp.size() <= 1) {
      printf("    -> 全部连通 (1 个域), 没有断开的帧\n");
      continue;
    }
    int main_root = comp.begin()->first;
    for (const auto& [r, v] : comp) {
      if (v.size() > comp[main_root].size()) main_root = r;
    }
    printf("    连通域 %zu 个 (最大的 %zu 帧):\n", comp.size(), comp[main_root].size());
    std::vector<ialign::Frame> keep;
    keep.reserve(n);
    std::size_t n_drop = 0;
    for (const auto& [r, v] : comp) {
      Eigen::Vector2d c(0, 0);
      for (const int i : v) c += S->frames[i].T_w_l.translation().head<2>();
      c /= static_cast<double>(v.size());
      // 集合到集合的最近距离 —— 不能用"质心到主域": 一条长路段的质心可能离主域几百米,
      // 而它的两端其实就挨着主域, 那个数会把人带偏。
      double dmin = 1e18;
      for (const int i : v) {
        for (const int j : comp[main_root])
          dmin = std::min(dmin, (S->frames[i].T_w_l.translation().head<2>() -
                                 S->frames[j].T_w_l.translation().head<2>()).norm());
      }
      // 空间尺度: 车根本没动的块(尺度~0)是停车原地扫描, 除了在图上糊成一坨没有任何价值;
      // 而尺度几百米的块是真实路段, 只是和主域之间缺了数据 —— 两者不能同等对待。
      double ext = 0;
      for (const int i : v) {
        for (const int j : v)
          ext = std::max(ext, (S->frames[i].T_w_l.translation().head<2>() -
                               S->frames[j].T_w_l.translation().head<2>()).norm());
      }
      std::set<std::string> clips;
      for (const int i : v) clips.insert(clip_of(S->frames[i].pcd_path));
      std::string cs;
      for (const auto& cn : clips) cs += (cs.empty() ? "" : ",") + cn.substr(cn.size() - 7);
      const bool want_drop = ext < static_ext;
      const bool drop = want_drop && do_drop;
      printf("      %s %4zu 帧 (%5.1f%%)  中心=(%8.1f,%8.1f)  离最大域 %7.1f m  自身尺度 %7.1f m"
             "  帧号 %d..%d  clip %s\n",
             want_drop ? (do_drop ? "**丢**" : "**本该丢,留**") : "  留  ", v.size(),
             100.0 * v.size() / n, c.x(), c.y(), dmin, ext, v.front(), v.back(), cs.c_str());
      if (want_drop) {
        printf("             ^ 这一块车没动过(尺度 %.2fm), 只是同一个位置扫了 %zu 遍 ——\n"
               "               在图上就是脱离主路线的一坨, 且拿不到有效的帧间约束%s\n",
               ext, v.size(), drop ? "。" : "; --drop_frames 0 下它照样进图, 位姿只有 INS 撑着。");
        if (drop) {
          n_drop += v.size();
          continue;
        }
      }
      for (const int i : v) keep.push_back(S->frames[i]);
    }
    printf("    -> 丢 %zu 帧, 剩 %zu 帧%s\n", n_drop, keep.size(),
           do_drop ? "" : "   (--drop_frames 0: 一帧都没丢)");
    // 帧必须保持原来的顺序(按 clip 再按时间) —— 后面分窗建帧间约束、按下标存位姿都靠它
    std::sort(keep.begin(), keep.end(),
              [](const ialign::Frame& a2, const ialign::Frame& b2) {
                return a2.pcd_path < b2.pcd_path;
              });
    S->frames.swap(keep);
  }
}

/// @brief 增量建图主流程: 按顺序把 N 个 session 并进来。
///
/// 每个待优化的 session 走三步:
///   阶段1  自身整段 BA        —— buildIntra(窗口内 GICP + 可选视觉) + buildLoop(回环)
///   阶段2  与前面 session 建约束 —— buildCross: 前面所有 session 拼 submap, 本 session 逐帧配上去
///   阶段3  自身 + 跨session 一起解 (前面的 session 此时固定)
/// 全部 session 处理完后再做一次**最终联合优化**: 所有位姿一起解, 只钉锚定帧。
///
/// 已优化的 session 从存档(或 --load_g2o)恢复位姿和约束, **不重新配准**; 但它们的位姿
/// 参与最终联合优化 —— 钉死会让新 session 只能单方面往老的上凑(实测 jjst2<->jjst3
/// 从 0.083 退化到 0.095), 而放开几乎不要钱(稀疏图解一次是秒级)。
///
/// @param S        已经统一到同一世界系、且做过孤立/静止块过滤的全部 session
/// @param base_utm 基准 session 的 utm_center, 写进存档用于校验同一世界系
/// @return 0 = 正常 (个别 session 失败只跳过它, 不整体失败)
static int runIncr(const Opt& o, std::vector<ialign::SessionData>& S,
                  const Eigen::Vector2d& base_utm, const ialign::CloudCovarianceEstimation& ce) {
  const int ns = static_cast<int>(S.size());
  // 存档目录**应当独立于 --out**: --out 里是几十 GB 的点云/las/dump, 反复实验时会被清掉,
  // 而存档是增量建图的**全部状态** (位姿 + 约束), 一起清掉就等于每次都从零重跑。
  // 默认值 <out>/poses 只是图省事, 真正用增量时一定要显式给一个独立目录。
  const fs::path store = o.pose_store.empty() ? (fs::path(o.out) / "poses") : fs::path(o.pose_store);
  {
    // 判断 store 是否落在 out 之内 (用规范化后的路径前缀比, 免得被 ./ 和 .. 骗过)
    std::error_code ec;
    const auto so = fs::weakly_canonical(fs::absolute(fs::path(o.out)), ec).string();
    const auto ss = fs::weakly_canonical(fs::absolute(store), ec).string();
    if (ss.compare(0, so.size(), so) == 0)
      printf("\n  !! 位姿存档在 --out 里面 (%s)\n"
             "     --out 会随实验被清掉, 存档跟着没了 = 每次都从零重跑, 增量白做。\n"
             "     建议: --pose_store <一个独立的持久目录>, 与 --out 分开。\n",
             store.string().c_str());
  }
  fs::create_directories(store);
  // 参数落盘 + 与存档比对。**在干活之前做** —— 跑几小时挂掉的话, 至少知道这次用了什么参数。
  {
    Opt& oo = const_cast<Opt&>(o);
    printf("\n=== 参数 ===\n");
    const bool conflict = diffParams(store / "params.yaml", oo);
    const fs::path eff = fs::path(o.out) / "params_effective.yaml";
    saveParams(eff, oo, "local_align 本次生效的参数");
    printf("  写出: %s\n", eff.string().c_str());
    // store/params.yaml 的语义是"**这批存档是用什么参数造出来的**"。
    // 配准类参数变了却仍复用旧存档时, 把新参数盖上去就是在撒谎 —— 存档里的约束仍是旧参数
    // 产物, 而且下次运行就再也比不出差异, 警告只响一次。所以这种情况**不覆盖**, 另存一份。
    if (conflict) {
      saveParams(store / "params_conflict.yaml", oo, "与存档参数冲突的一次运行");
      printf("  !! 存档的 params.yaml **保持原样**(它记录存档实际是怎么造的);\n"
             "     本次参数另存为 %s\n", (store / "params_conflict.yaml").string().c_str());
    } else {
      saveParams(store / "params.yaml", oo, "local_align 生成此存档所用的参数");
      printf("  写出: %s\n", (store / "params.yaml").string().c_str());
    }
  }

  // ---- 全局参数下标: 各 session 首尾相接 ----
  std::vector<int> ofs(ns + 1, 0);
  for (int k = 0; k < ns; k++) ofs[k + 1] = ofs[k] + static_cast<int>(S[k].frames.size());
  const int ntot = ofs[ns];

  // 原始(yaml)外参: 下面会把已优化 session 的 S[k].T_v_l 换成优化后的共享 ext, 而
  // _before 那一份导出和 poses_cmp.csv 的 "INS 那一列" 要的是**原始**外参, 所以先留一份。
  std::vector<Eigen::Isometry3d> Tvl0(ns);
  for (int k = 0; k < ns; k++) Tvl0[k] = S[k].T_v_l;

  // ---- 扫存档: 明确列出哪些 session 已经优化好 ----
  const auto splitCsv = [](const std::string& c) {
    std::vector<std::string> v;
    std::size_t p = 0;
    while (p <= c.size()) {
      const auto q = c.find(',', p);
      const std::string t = c.substr(p, q == std::string::npos ? std::string::npos : q - p);
      if (!t.empty()) v.push_back(t);
      if (q == std::string::npos) break;
      p = q + 1;
    }
    return v;
  };
  const auto redo = splitCsv(o.redo);
  printf("\n=== session 清单 (存档目录 %s) ===\n", store.string().c_str());
  std::vector<char> done(ns, 0);
  std::vector<PoseRec> rec(ns);
  for (int k = 0; k < ns; k++) {
    const fs::path f = store / (S[k].name + ".csv");
    const bool forced = std::find(redo.begin(), redo.end(), S[k].name) != redo.end();
    std::string why;
    if (forced) {
      why = "--redo 指定重做";
    } else if (!fs::exists(f)) {
      why = "无存档";
    } else if (!loadPoses(f, rec[k])) {
      why = "存档读不出来";
    } else if (rec[k].base_utm != utmTag(base_utm)) {
      why = "存档的 base_utm=" + rec[k].base_utm + " 与本次 " + utmTag(base_utm) + " 不一致";
    } else {
      // 必须**每一帧**都能按路径找到位姿, 否则宁可重做也不要半套位姿
      std::size_t hit = 0;
      for (const auto& fr : S[k].frames) hit += rec[k].by_path.count(fr.pcd_path);
      if (hit != S[k].frames.size()) {
        char b[160];
        std::snprintf(b, sizeof(b), "存档只覆盖 %zu/%zu 帧 (ROI 变了?)", hit, S[k].frames.size());
        why = b;
      } else {
        done[k] = 1;
      }
    }
    printf("  [%d] %-34s 帧=%-5zu %s\n", k, S[k].name.c_str(), S[k].frames.size(),
           done[k] ? "**已优化, 本次跳过**" : ("待优化 (" + why + ")").c_str());
  }
  // ---- 载入已有位姿图 ----
  // 里面覆盖到的 session 视为已优化: 约束直接复用(不重新构造), 位姿参与最终联合优化。
  LoadedGraph lg;
  if (!o.load_g2o.empty()) {
    printf("\n=== 载入已有位姿图 ===\n");
    if (!loadG2o(o.load_g2o, lg)) {
      std::cerr << "  !! 位姿图载入失败\n";
      return 1;
    }
    for (int k = 0; k < ns; k++) {
      if (!lg.sessions.count(S[k].name)) continue;
      std::size_t hit = 0;
      for (const auto& fr : S[k].frames) hit += lg.pose_of.count(fr.pcd_path);
      if (hit != S[k].frames.size()) {
        printf("    %-34s 图里只覆盖 %zu/%zu 帧 —— **不当作已优化**, 本次重算\n",
               S[k].name.c_str(), hit, S[k].frames.size());
        continue;
      }
      done[k] = 1;
      printf("    %-34s %zu 帧全部命中 -> 已优化, 约束复用\n", S[k].name.c_str(), hit);
    }
  }

  const std::vector<char> done_before = done;   // 本次开始前就已优化的 (用于报告)
  if (o.list_done) {
    printf("\n  --list_done: 只列清单, 不干活\n");
    return 0;
  }
  if (!done[0] && ns > 1) printf("\n  注意: 基准 session [0] 尚未优化, 本次会先优化它\n");

  // ---- 位姿状态 ----
  std::vector<Eigen::Isometry3d> Tins(ntot), Tcur(ntot);   // 车体位姿: INS 初值 / 当前
  for (int k = 0; k < ns; k++) {
    for (std::size_t i = 0; i < S[k].frames.size(); i++) {
      Tins[ofs[k] + i] = S[k].frames[i].T_w_v;
      Tcur[ofs[k] + i] = S[k].frames[i].T_w_v;
    }
  }
  Eigen::Isometry3d ext = S[0].T_v_l;
  if (lg.has_ext) {
    const Eigen::AngleAxisd da(ext.linear().transpose() * lg.ext.linear());
    printf("  外参取自载入的位姿图 (与 yaml 初值差 %.4f 度, %.4f m)\n",
           std::abs(da.angle()) * 180.0 / M_PI, (lg.ext.translation() - ext.translation()).norm());
    ext = lg.ext;
  }
  // 已优化的 session 直接用存档位姿; 外参取第一个存档里的(同一辆车, 外参是车的属性)
  bool ext_from_store = lg.has_ext;   // 载入的图已经给了外参就不再从 csv 取
  for (int k = 0; k < ns; k++) {
    if (!done[k]) continue;
    // **只有 csv 存档真的读出了位姿才用它**。--load_g2o 会把 done 置 1, 而此时 rec[k]
    // 可能是空的 —— 空 map 上用 operator[] 会当场插入一个**单位阵**并返回, 于是位姿和
    // 外参被静默改成单位阵。实测就是这个让老 session 在联合优化里被挪了 3.56m。
    if (rec[k].by_path.empty()) continue;
    for (std::size_t i = 0; i < S[k].frames.size(); i++) {
      const auto it = rec[k].by_path.find(S[k].frames[i].pcd_path);
      if (it != rec[k].by_path.end()) Tcur[ofs[k] + i] = it->second;
    }
    if (!ext_from_store) {
      ext = rec[k].ext;
      ext_from_store = true;
      printf("  外参取自 %s 的存档\n", S[k].name.c_str());
    }
  }
  // ---- 把存档里的优化后位姿**写回 S[k].frames** ----
  // 之前 Tcur 是优化后的、而 S[k].frames[i].T_w_l 永远停在原始 INS, 同一个概念(世界系
  // 雷达位姿)在内存里有两份且不一致 —— 实测差中位 0.16~0.17m, 比我们在追的所有误差都大,
  // 而且用错那份不报错。这个项目已被同类"两份真相"坑过两次(--load_roi 的下标错位、
  // 空 map 上 operator[] 静默插入单位阵)。
  // **只写已优化的 session**: 第一次参与优化的 session 必须保持原始 INS —— 那正是它自己
  // 那轮 BA 的初值。
  // 顺序要求: 必须在 Tins 填完**之后**。Tins 是最终联合优化的先验**测量值**, 它测的是
  // 当初 INS 给出的位置; 换成优化结果就变成"把位姿钉在自己身上"的自证约束。
  {
    std::size_t nwb = 0;
    int nsess_wb = 0;
    for (int k = 0; k < ns; k++) {
      if (!done[k] || rec[k].by_path.empty()) continue;
      for (std::size_t i = 0; i < S[k].frames.size(); i++) {
        S[k].frames[i].T_w_v = Tcur[ofs[k] + i];
        S[k].frames[i].T_w_l = Tcur[ofs[k] + i] * ext;   // 外参也换成优化后的共享 ext
        nwb++;
      }
      // 保持 T_w_l == T_w_v * T_v_l 这个不变量, 否则又造出一处不一致
      S[k].T_v_l = ext;
      nsess_wb++;
    }
    if (nwb)
      printf("  已优化的 %d 个 session、%zu 帧的位姿已写回内存 —— 之后所有路径读到的都是"
             "**优化后**的位姿 (新 session 仍用原始 INS 当初值)\n", nsess_wb, nwb);
  }
  // ---- 把优化后的位姿**写回 S[k].frames**, 让内存里只有一套真相 ----
  // 之前 Tcur 是优化后的、而 S[k].frames[i].T_w_l 永远是原始 INS, 同一个概念(世界系雷达
  // 位姿)存在两份且不一致 —— 实测差中位 0.16~0.17m, 比我们在追的所有误差都大, 而且用错
  // 那份不会报错。这个项目已经被同类的"两份真相"坑过两次(--load_roi 的下标错位、
  // 空 map 上 operator[] 静默插入单位阵)。
  // **只对已优化的 session 写回**: 第一次参与优化的 session 必须保持原始 INS, 那正是它
  // 自己 BA 的初值。
  // 顺序要求: 必须在 Tins 填完**之后** —— Tins 是最终联合优化的先验**测量值**, 它测的是
  // 当初 INS 给的位置, 换成优化结果就变成"把位姿钉在自己身上"的自证约束。
  {
    std::size_t nwb = 0;
    for (int k = 0; k < ns; k++) {
      if (!done[k] || rec[k].by_path.empty()) continue;
      for (std::size_t i = 0; i < S[k].frames.size(); i++) {
        S[k].frames[i].T_w_v = Tcur[ofs[k] + i];
        S[k].frames[i].T_w_l = Tcur[ofs[k] + i] * ext;   // 外参也换成优化后的共享 ext
        nwb++;
      }
      // 保持 T_w_l = T_w_v * T_v_l 这个不变量成立, 否则又造出一处不一致
      S[k].T_v_l = ext;
    }
    if (nwb)
      printf("  已优化 session 的位姿已写回内存 (%zu 帧): 之后所有路径读到的都是**优化后**的\n",
             nwb);
  }

  ialign::SubmapCeresOpts co;
  co.sigma_ins_xy = o.ba_ins_xy;
  co.sigma_rel_t = o.ba_rel_t;
  co.sigma_rel_r = o.ba_rel_r;
  co.attitude_w = o.ba_att_w;
  co.huber = o.ba_huber;
  co.iters = o.ba_iters;
  co.sigma_px = o.ba_sigma_px;
  co.opt_ext = o.opt_ext;

  // 全程累积: 画约束图 / 写 g2o 用。跨 session 边是唯一把两趟绑在一起的东西,
  // 所以要留着看它到底分布在哪儿。
  std::vector<GraphEdge> all_cross;
  std::vector<std::tuple<int, int, Eigen::Isometry3d>> all_intra;
  // 与 all_intra 等长: 0=窗口内 2=同向回环 3=反向回环。最终联合优化要靠它分 sigma,
  // 否则两百条回环边会按窗口边的 0.035 加权 —— 那个值是为相邻同向帧对校准的。
  std::vector<int> all_intra_kind;
  // pcd 路径 -> 全局下标, 用来把存档里的边还原成下标
  std::map<std::string, int> gidx_of;
  std::vector<int> sess_of_g(ntot, 0);
  for (int k = 0; k < ns; k++) {
    for (std::size_t i = 0; i < S[k].frames.size(); i++) {
      gidx_of[S[k].frames[i].pcd_path] = ofs[k] + static_cast<int>(i);
      sess_of_g[ofs[k] + i] = k;
    }
  }
  std::vector<char> fixed_final(ntot, 0);
  for (int k = 0; k < ns; k++) {
    if (!done[k]) continue;   // 本次开始前就已优化的 -> 它们是固定参照
    for (int i = ofs[k]; i < ofs[k + 1]; i++) fixed_final[i] = 1;
  }

  // 载入图里的位姿/边 (优先于 csv 存档: 用户显式指定的那份为准)
  if (!lg.pose_of.empty()) {
    std::size_t nv = 0;
    for (int k = 0; k < ns; k++) {
      if (!done[k] || !lg.sessions.count(S[k].name)) continue;
      for (std::size_t i = 0; i < S[k].frames.size(); i++) {
        const auto it = lg.pose_of.find(S[k].frames[i].pcd_path);
        if (it == lg.pose_of.end()) continue;
        Tcur[ofs[k] + i] = it->second;
        nv++;
      }
    }
    std::size_t ne = 0, nc = 0;
    for (const auto& [pa, pb, T] : lg.edges) {
      const auto a1 = gidx_of.find(pa), b1 = gidx_of.find(pb);
      if (a1 == gidx_of.end() || b1 == gidx_of.end()) continue;
      const int si = sess_of_g[a1->second], sj = sess_of_g[b1->second];
      if (si == sj) {
        all_intra.emplace_back(a1->second, b1->second, T);
        all_intra_kind.push_back(0);   // g2o 格式里没有回环标记, 一律按窗口边加权
        ne++;
      } else {
        all_cross.push_back({a1->second, b1->second, si, sj, T, -1.0});
        nc++;
      }
    }
    printf("    -> 位姿 %zu 个, 同session边 %zu, 跨session边 %zu 已并入本次的图\n", nv, ne, nc);
  }

  // 已优化 session 的约束从存档读回来 —— 否则跳过它就等于把它的边丢了, 约束图会缺一大块
  for (int k = 0; k < ns; k++) {
    if (!done[k]) continue;
    if (lg.sessions.count(S[k].name)) continue;   // 已由 --load_g2o 提供, 别重复加
    std::vector<EdgeRec> es;
    const fs::path ef = store / (S[k].name + "_edges.csv");
    if (!fs::exists(ef) || !loadEdges(ef, es)) {
      printf("  [约束存档] %s 没有边存档 —— 约束图里会缺它这部分 (用 --redo %s 重算)\n",
             S[k].name.c_str(), S[k].name.c_str());
      continue;
    }
    std::size_t miss = 0;
    for (const auto& e : es) {
      const auto a1 = gidx_of.find(e.from), b1 = gidx_of.find(e.to);
      if (a1 == gidx_of.end() || b1 == gidx_of.end()) { miss++; continue; }
      if (e.kind == 0 || e.kind == 2 || e.kind == 3) {   // 0=窗口内 2/3=回环, 都是同 session 边
        all_intra.emplace_back(a1->second, b1->second, e.T);
        all_intra_kind.push_back(e.kind);
      } else {
        all_cross.push_back({a1->second, b1->second, sess_of_g[a1->second],
                             sess_of_g[b1->second], e.T, e.nn, e.ss});
      }
    }
    printf("  [约束存档] %s 读回 %zu 条边%s\n", S[k].name.c_str(), es.size() - miss,
           miss ? (" (有 " + std::to_string(miss) + " 条的端点不在本次帧集里, 已丢弃)").c_str() : "");
  }

  std::vector<ialign::CamModel> vcam;
  if (o.ba_visual) {
    for (int k = 0; k < ns && vcam.empty(); k++) {
      if (S[k].frames.empty()) continue;
      vcam = ialign::loadCameras(S[k].root, o.data_mode, S[k].T_v_l, o.vis_cams,
                                 S[k].frames[0].pcd_path);
    }
    printf("  [视觉] 相机 %zu 个:", vcam.size());
    for (const auto& c : vcam) printf(" %s", c.name.c_str());
    printf("\n");
  }

  // ---- 依次并入 ----
  for (int k = 0; k < ns; k++) {
    if (done[k]) continue;
    g_io.mark();   // 这个 session 的读取成败单独算 (见 guardIo)
    printf("\n############ 并入 session[%d] %s (%zu 帧) ############\n", k, S[k].name.c_str(),
           S[k].frames.size());
    const int g0 = ofs[k], nk = static_cast<int>(S[k].frames.size());
    if (nk < 5) {
      printf("  帧太少, 跳过\n");
      continue;
    }
    std::vector<int> idx(nk);
    for (int i = 0; i < nk; i++) idx[i] = i;

    // 参照系: 前面所有**已优化**的 session 的帧
    std::vector<RefFrame> ref;
    for (int j = 0; j < k; j++) {
      if (!done[j]) continue;
      for (std::size_t i = 0; i < S[j].frames.size(); i++) {
        ref.push_back({ofs[j] + static_cast<int>(i), j, Tcur[ofs[j] + i] * ext,
                       S[j].frames[i].pcd_path});
      }
    }

    // ---- 阶段1: 本 session 自己整段 BA ----
    printf("\n=== [%s] 阶段1: 自身整段 BA ===\n", S[k].name.c_str());
    auto ck = buildIntra(S[k], idx, g0, o, ce, vcam, S[k].name.c_str());
    if (ck.rel.empty()) {
      printf("  !! 一条帧间约束都没建出来, 跳过这个 session\n");
      continue;
    }
    // ---- 回环: 独立的一路, 结果并进同一批帧间约束 ----
    const std::size_t n_win = ck.rel.size();
    if (o.loop) {
      const auto cl = buildLoop(S[k], idx, g0, o, ce, S[k].name.c_str());
      ck.rel.insert(ck.rel.end(), cl.rel.begin(), cl.rel.end());
      ck.kind.insert(ck.kind.end(), cl.kind.begin(), cl.kind.end());
    }
    const std::size_t n_loop = ck.rel.size() - n_win;
    if (ck.kind.size() != ck.rel.size()) ck.kind.assign(ck.rel.size(), 0);   // 兜底
    std::vector<char> fixed(ntot, 1);
    for (int i = 0; i < nk; i++) fixed[g0 + i] = 0;   // 只有本 session 自由
    all_intra.insert(all_intra.end(), ck.rel.begin(), ck.rel.end());
    all_intra_kind.insert(all_intra_kind.end(), ck.kind.begin(), ck.kind.end());
    {
      const std::vector<int>& grp = ck.kind;   // 0=窗口内 2=同向回环 3=反向回环
      printf("  约束: 帧间 %zu 条 (窗口内 %zu + 回环 %zu), 视觉 %zu 条\n", ck.rel.size(), n_win,
             n_loop, ck.vis.size());
      std::vector<Eigen::Isometry3d> out;
      Eigen::Isometry3d e1 = ext;
      CalibIn ci{&Tcur, &ext, &ck.rel, &ck.vis, &grp, &fixed};
      // 基准 session 定规范: 锚第 0 帧; 后续 session 的规范由固定的参照帧给出
      const auto st = solveCalib(ci, co, o.calib_rounds, o.cross_sigma_xy, o.cross_sigma_r, out, e1,
                                "阶段1", o.calib_priors != 0, ref.empty() ? g0 : -1,
                                o.loop_sigma_t, o.loop_sigma_r, o.loop_anti_w);
      if (!st.ok) {
        printf("  !! 阶段1 求解失败, 跳过\n");
        continue;
      }
      std::vector<double> d(nk);
      for (int i = 0; i < nk; i++)
        d[i] = (out[g0 + i].translation() - Tcur[g0 + i].translation()).norm();
      std::vector<double> ds = d;
      std::sort(ds.begin(), ds.end());
      printf("  阶段1 位移: 中位=%.3f p90=%.3f max=%.3f m  (固定帧 %d)\n", ds[ds.size() / 2],
             ds[ds.size() * 9 / 10], ds.back(), st.n_fixed);
      // 位移最大的几帧: 单看中位/p90 看不出有没有帧被甩飞。一帧被优化挪了上百米,
      // 中位数完全不受影响 —— 必须单独把尾巴报出来。
      {
        std::vector<int> ord(nk);
        for (int i = 0; i < nk; i++) ord[i] = i;
        std::partial_sort(ord.begin(), ord.begin() + std::min(nk, 6), ord.end(),
                          [&](int a1, int b1) { return d[a1] > d[b1]; });
        printf("    位移最大的 %d 帧:\n", std::min(nk, 6));
        for (int u = 0; u < std::min(nk, 6); u++) {
          const int i = ord[u];
          const auto t0 = (Tcur[g0 + i] * ext).translation();
          const auto t1 = (out[g0 + i] * ext).translation();
          printf("      #%-5d 位移=%8.3f m  (%8.1f,%8.1f,%6.1f) -> (%8.1f,%8.1f,%6.1f)  %s\n", i,
                 d[i], t0.x(), t0.y(), t0.z(), t1.x(), t1.y(), t1.z(),
                 fs::path(S[k].frames[i].pcd_path).filename().string().c_str());
        }
      }
      for (int i = 0; i < nk; i++) Tcur[g0 + i] = out[g0 + i];
      if (ref.empty()) ext = e1;   // 外参只在基准 session 上估一次, 之后当车的固有属性用
    }

    // ---- 阶段2/3: 并到前面已优化的 session 上 ----
    if (ref.empty()) {
      printf("\n  这是基准 session, 没有可并入的参照, 到此为止\n");
    } else {
      printf("\n=== [%s] 阶段2: 与前面 session 的重叠区域建约束 ===\n", S[k].name.c_str());
      std::vector<Eigen::Isometry3d> Tk_l(nk);
      for (int i = 0; i < nk; i++) Tk_l[i] = Tcur[g0 + i] * ext;
      const auto cc = buildCross(ref, S[k], g0, Tk_l, o, ce);
      auto med = [](std::vector<double> v) {
        if (v.empty()) return -1.0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
      };
      printf("  重叠区域=%d  尝试配准=%d  建成约束=%zu\n"
             "  区域内 nn: 配准前 中位=%.3f -> 配准后 中位=%.3f m\n",
             cc.n_region, cc.n_try, cc.rel.size(), med(cc.nn0), med(cc.nn1));
      // 记进全局表, 顺便记下每条边两端属于哪个 session (画图要按 session 上色)
      for (std::size_t q = 0; q < cc.rel.size(); q++) {
        const auto& [gi, gj, T] = cc.rel[q];
        int si = 0;
        while (si + 1 < ns && gi >= ofs[si + 1]) si++;
        all_cross.push_back({gi, gj, si, k, T, q < cc.nn1.size() ? cc.nn1[q] : -1.0,
                             q < cc.sig_scale.size() ? cc.sig_scale[q] : 1.0});
      }
      if (cc.rel.empty()) {
        printf("  !! 没建出跨 session 约束, 这个 session 只有自身 BA 的结果\n");
      } else {
        printf("\n=== [%s] 阶段3: 联合优化 (自身 + 跨session, 前面的 session 固定) ===\n",
               S[k].name.c_str());
        std::vector<std::tuple<int, int, Eigen::Isometry3d>> all = ck.rel;
        std::vector<int> grp = ck.kind;
        std::vector<double> sscale(ck.rel.size(), 1.0);
        all.insert(all.end(), cc.rel.begin(), cc.rel.end());
        grp.resize(all.size(), 1);
        for (std::size_t u = 0; u < cc.rel.size(); u++)
          sscale.push_back(u < cc.sig_scale.size() ? cc.sig_scale[u] : 1.0);
        printf("  约束: 自身帧间 %zu (sigma t=%.3f) + 跨session %zu (sigma t=%.3f) + 视觉 %zu\n",
               ck.rel.size(), o.ba_rel_t, cc.rel.size(), o.cross_sigma_xy, ck.vis.size());
        std::vector<Eigen::Isometry3d> out;
        Eigen::Isometry3d e2 = ext;
        const auto T1 = Tcur;
        CalibIn ci{&T1, &ext, &all, &ck.vis, &grp, &fixed, nullptr, &sscale};
        const auto st = solveCalib(ci, co, o.calib_rounds, o.cross_sigma_xy, o.cross_sigma_r, out,
                                  e2, "阶段3", o.calib_priors != 0, -1, o.loop_sigma_t,
                                  o.loop_sigma_r, o.loop_anti_w);
        if (!st.ok) {
          printf("  !! 阶段3 求解失败, 保留阶段1 的结果\n");
        } else {
          std::vector<double> d;
          for (int i = 0; i < nk; i++)
            d.push_back((out[g0 + i].translation() - T1[g0 + i].translation()).norm());
          std::sort(d.begin(), d.end());
          printf("  阶段3 相对阶段1 的位移: 中位=%.3f p90=%.3f m\n", d[d.size() / 2],
                 d[d.size() * 9 / 10]);
          for (int i = 0; i < nk; i++) Tcur[g0 + i] = out[g0 + i];
        }
      }
    }

    // ---- 存档: 存完这个 session 才算"已优化" ----
    std::vector<Eigen::Isometry3d> Tv(nk);
    for (int i = 0; i < nk; i++) Tv[i] = Tcur[g0 + i];
    const fs::path pf = store / (S[k].name + ".csv");
    // 写存档之前守门: 这个 session 读失败太多就中止, 绝不留下一份看不出问题的坏存档
    guardIo(o.max_io_fail, S[k].name.c_str(), true);
    if (savePoses(pf, S[k], Tv, ext, base_utm)) {
      done[k] = 1;
      printf("  存档: %s\n", pf.string().c_str());
    } else {
      printf("  !! 存档写失败: %s\n", pf.string().c_str());
    }
    // 优化前后的位姿对照表 (拿它画分布 / 找被甩飞的帧)
    {
      std::ofstream cf((fs::path(o.out) / (S[k].name + "_poses_cmp.csv")).string());
      cf << "idx,x_ins,y_ins,z_ins,x_opt,y_opt,z_opt,disp,nn_ins,pcd\n";
      cf.precision(10);
      for (int i = 0; i < nk; i++) {
        const auto a0 = (Tins[g0 + i] * Tvl0[k]).translation();
        const auto a1 = (Tcur[g0 + i] * ext).translation();
        double nn = 1e18;
        for (int j2 = 0; j2 < nk; j2++) {
          if (j2 == i) continue;
          nn = std::min(nn, ((Tins[g0 + j2] * Tvl0[k]).translation() - a0).norm());
        }
        cf << i << ',' << a0.x() << ',' << a0.y() << ',' << a0.z() << ',' << a1.x() << ','
           << a1.y() << ',' << a1.z() << ',' << (a1 - a0).norm() << ',' << nn << ','
           << S[k].frames[i].pcd_path << '\n';
      }
      printf("  位姿对照: %s\n", (S[k].name + "_poses_cmp.csv").c_str());
    }

    // 约束也存: 下次跳过这个 session 时才画得出完整的约束图
    {
      std::vector<EdgeRec> es;
      const auto path_of = [&](int g) {
        const int sk = sess_of_g[g];
        return S[sk].frames[g - ofs[sk]].pcd_path;
      };
      // 回环边记成 kind=2, 载回来时还能区分 (求解时和窗口内边同等对待)
      for (std::size_t u = 0; u < ck.rel.size(); u++) {
        const auto& [gi, gj, T] = ck.rel[u];
        es.push_back({path_of(gi), path_of(gj), T, -1.0,
                      u < ck.kind.size() ? ck.kind[u] : (u < n_win ? 0 : 2)});
      }
      for (const auto& e : all_cross) {
        if (e.sj != k) continue;
        es.push_back({path_of(e.gi), path_of(e.gj), e.T, e.nn, 1, e.ss});
      }
      const fs::path ef = store / (S[k].name + "_edges.csv");
      if (saveEdges(ef, es)) printf("  约束存档: %s (%zu 条)\n", ef.string().c_str(), es.size());
    }
  }

  // ---- 最终联合优化: 所有 session 的位姿一起解, 只钉锚定帧 ----
  // 这一步是增量方案的关键: 老 session 的**约束**复用(不重算), 但它们的**位姿**参与优化。
  // 钉死老位姿的代价实测过 —— ROI 里 jjst2<->jjst3 从 0.083 退化到 0.095, 因为新 session
  // 只能单方面往老的上凑, 而两边各有误差时正确解是各让一半。
  // 放开几乎不要钱: 稀疏图解一次是秒级, 真正的成本在前面的配准。
  if (!o.freeze_prev && !all_cross.empty() && ns > 1) {
    printf("\n=== 最终联合优化 (全部 session 位姿一起解, 约束复用) ===\n");
    std::vector<std::tuple<int, int, Eigen::Isometry3d>> all = all_intra;
    std::vector<int> grp = all_intra_kind;
    grp.resize(all_intra.size(), 0);
    std::vector<double> sscale(all_intra.size(), 1.0);
    for (const auto& e : all_cross) {
      all.emplace_back(e.gi, e.gj, e.T);
      sscale.push_back(e.ss);
    }
    grp.resize(all.size(), 1);
    printf("  约束: 同session 帧间 %zu + 跨session %zu  |  位姿 %d 个全部可动 (锚定 #0)\n",
           all_intra.size(), all_cross.size(), ntot);
    const auto Tprev = Tcur;
    std::vector<ialign::VisReprojSpecExt> novis;
    std::vector<Eigen::Isometry3d> out;
    Eigen::Isometry3d e3 = ext;
    // 先验的**测量值**一律用 INS —— 不能用上一轮的优化结果, 那会变成把位姿钉在自己身上
    CalibIn ci{&Tprev, &ext, &all, &novis, &grp, nullptr, &Tins, &sscale};
    ialign::SubmapCeresOpts cf = co;
    cf.opt_ext = 0;   // 外参已在基准 session 上定过, 这里不再动
    const auto st = solveCalib(ci, cf, o.calib_rounds, o.cross_sigma_xy, o.cross_sigma_r, out, e3,
                              "联合", o.calib_priors != 0, 0, o.loop_sigma_t, o.loop_sigma_r,
                              o.loop_anti_w);
    if (!st.ok) {
      printf("  !! 联合优化失败, 保留各 session 各自的结果\n");
    } else {
      for (int k = 0; k < ns; k++) {
        std::vector<double> d;
        for (int i = ofs[k]; i < ofs[k + 1]; i++)
          d.push_back((out[i].translation() - Tprev[i].translation()).norm());
        if (d.empty()) continue;
        std::sort(d.begin(), d.end());
        printf("    %-34s 位移 中位=%.3f p90=%.3f max=%.3f m%s\n", S[k].name.c_str(),
               d[d.size() / 2], d[d.size() * 9 / 10], d.back(),
               done_before[k] ? "   <- 老 session, 这次也被修正了" : "");
      }
      Tcur = out;
      // 位姿变了, 存档要跟着更新, 否则下次载入的是旧值
      for (int k = 0; k < ns; k++) {
        std::vector<Eigen::Isometry3d> Tv(S[k].frames.size());
        for (std::size_t i = 0; i < S[k].frames.size(); i++) Tv[i] = Tcur[ofs[k] + i];
        guardIo(o.max_io_fail, "最终联合优化", false);
        savePoses(store / (S[k].name + ".csv"), S[k], Tv, ext, base_utm);
      }
      printf("  位姿存档已按联合优化的结果更新\n");
    }
  }

  // ---- manifest ----
  {
    std::ofstream mf((store / "manifest.txt").string());
    mf << "# 增量建图位姿存档。有 <name>.csv 且 base_utm 对得上, 该 session 就算已优化。\n";
    mf << "# base_utm=" << utmTag(base_utm) << "\n";
    for (int k = 0; k < ns; k++)
      mf << (done[k] ? "done  " : "todo  ") << S[k].name << "  frames=" << S[k].frames.size()
         << "\n";
  }

  // ---- 约束图 + g2o ----
  printf("\n=== 约束图 / g2o (写到位姿存档目录 %s —— 它们和位姿/约束是一套状态,\n    应当和点云那些一次性产物分开保存) ===\n", store.string().c_str());
  {
    std::vector<Eigen::Isometry3d> Pl(ntot);
    std::vector<std::string> names;
    for (int k = 0; k < ns; k++) {
      names.push_back(S[k].name);
      for (int i = ofs[k]; i < ofs[k + 1]; i++) Pl[i] = Tcur[i] * ext;
    }
    const std::vector<int>& sess_of = sess_of_g;
    // 跨 session 边按 session 对统计 —— "建成 N 条"看不出这 N 条是铺满全程还是挤在几处
    std::map<std::pair<int, int>, int> per_pair;
    for (const auto& e : all_cross) per_pair[{e.si, e.sj}]++;
    for (const auto& [pr, n] : per_pair)
      printf("  跨session 约束 %s -> %s : %d 条\n", S[pr.first].name.c_str(),
             S[pr.second].name.c_str(), n);
    drawConstraintGraph(Pl, sess_of, names, all_cross, store / "constraint_graph.png");
    // 放大图: 自动挑约束最密的地方 (100m 格计数)
    if (!all_cross.empty()) {
      std::map<std::pair<int, int>, int> dens;
      for (const auto& e : all_cross) {
        const auto& t = Pl[e.gj].translation();
        dens[{static_cast<int>(std::floor(t.x() / 100)), static_cast<int>(std::floor(t.y() / 100))}]++;
      }
      auto best = dens.begin();
      for (auto it = dens.begin(); it != dens.end(); ++it) {
        if (it->second > best->second) best = it;
      }
      const Eigen::Vector3d z(best->first.first * 100.0 + 50, best->first.second * 100.0 + 50, 160.0);
      printf("  约束最密的地方: (%.0f, %.0f) 附近 %d 条 -> 放大图\n", z.x(), z.y(), best->second);
      drawConstraintGraph(Pl, sess_of, names, all_cross,
                          store / "constraint_graph_zoom.png", 1600, &z);
    }
    saveG2o(store / "pose_graph.g2o", Tcur, sess_of, fixed_final, all_intra, o.ba_rel_t,
            o.ba_rel_r, all_cross, o.cross_sigma_xy, o.cross_sigma_r, Tins, o.ba_ins_xy, 0.75,
            o.ba_att_w, 0, names);
    {
      std::vector<std::string> paths(ntot);
      for (int k = 0; k < ns; k++) {
        for (std::size_t i = 0; i < S[k].frames.size(); i++)
          paths[ofs[k] + i] = S[k].frames[i].pcd_path;
      }
      if (saveG2oIndex(store / "pose_graph_index.csv", sess_of, names, paths, ext))
        printf("    索引: pose_graph_index.csv (顶点 id -> pcd 路径; --load_g2o 要用它)\n");
    }
    // 边的清单: 拿它按质量排序找配坏的地方
    std::ofstream ef((store / "cross_edges.csv").string());
    ef << "sess_from,sess_to,g_from,g_to,x_from,y_from,x_to,y_to,dx,dy,dist,nn_after\n";
    for (const auto& e : all_cross) {
      const auto a2 = Pl[e.gi].translation(), b2 = Pl[e.gj].translation();
      ef << S[e.si].name << ',' << S[e.sj].name << ',' << e.gi << ',' << e.gj << ',' << a2.x() << ','
         << a2.y() << ',' << b2.x() << ',' << b2.y() << ',' << (b2.x() - a2.x()) << ','
         << (b2.y() - a2.y()) << ',' << (b2 - a2).head<2>().norm() << ',' << e.nn << '\n';
    }
  }

  // ---- 导出 ----
  if (!o.do_export) {
        printf("\n=== 导出点云: **跳过** (--export 0) ===\n");
    return 0;
  }
// 导出阶段每帧只顺序读一次, 缓存命中率为零 —— 留着纯粹和体素累积争内存
  // (全量体素那一路要把所有点收进内存, 12000 帧量级是 10 GB)。先报配准阶段的命中率再清空。
  g_fcache.report("配准阶段");
  g_fcache.clear();
  printf("\n=== 导出点云 ===\n");
  std::vector<ialign::CamModel> ccam;
  if (o.color) {
    for (int k = 0; k < ns && ccam.empty(); k++) {
      if (S[k].frames.empty()) continue;
      ccam = ialign::loadCameras(S[k].root, o.data_mode, S[k].T_v_l, o.color_cams,
                                 S[k].frames[0].pcd_path);
    }
    printf("  [上色] 相机 %zu 个:", ccam.size());
    for (const auto& c : ccam) printf(" %s", c.name.c_str());
    printf("\n");
  }
  for (int k = 0; k < ns; k++) {
    for (int pass = 0; pass < 2; pass++) {
      const bool after = pass == 1;
      if (!after && !o.export_before) continue;   // --export_before 0: 只导 _after
      const auto& Tv = after ? Tcur : Tins;
      const Eigen::Isometry3d& e = after ? ext : Tvl0[k];   // _before 要**原始**外参
      const std::string nm = S[k].name + (after ? "_after" : "_before");
      const int nk = static_cast<int>(S[k].frames.size());
      const bool with_rgb = !ccam.empty();
      VoxelAcc acc;
      acc.res = o.submap_voxel;
      ColorStat tot;
      // ---- 三份输出, 三种处理深度 ----
      //   .pcd      体素滤波后 (VoxelAcc) —— **量重影/厚度的基准, 必须始终是这个**,
      //             换成原始点就和之前所有测量不可比
      //   .las      export_raw=1 时是**原始点流式写**; 否则和 .pcd 同一份体素结果
      //   _rgb.pcd  同上
      // 原始点那两份走流式写盘: 全量 1.69 亿点先攒后写要 7GB+ 内存, 会被 OOM 杀掉。
      ialign::LasStream lasS;
      ialign::PcdStream rgbS;
      const bool raw_mode = o.export_raw != 0;
      bool las_ok = false, rgb_ok = false;
      if (raw_mode) {
        if (o.las)
          las_ok = lasS.open((fs::path(o.out) / (nm + ".las")).string(), base_utm, o.utm_zone);
        if (with_rgb)
          rgb_ok = rgbS.open((fs::path(o.out) / (nm + "_rgb.pcd")).string(), true);
      }
      // 分块: 一块算完就并进体素格/写盘再释放, 峰值内存只和分块大小有关。
      // 原始点模式下每帧 6.7 万点(体素后 3 万), 所以块要小一半。
      const int chunk = raw_mode ? 100 : 200;
      for (int b0 = 0; b0 < nk; b0 += chunk) {
        const int b1 = std::min(nk, b0 + chunk);
        std::vector<std::vector<Eigen::Vector4d>> fp(b1 - b0), fpr(b1 - b0);
        std::vector<std::vector<Rgb>> fc(b1 - b0), fcr(b1 - b0);
        std::vector<std::vector<char>> fo(b1 - b0), forr(b1 - b0);
        std::vector<std::vector<double>> fi(b1 - b0), fir(b1 - b0);
        std::vector<ColorStat> fs_(b1 - b0);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
        for (int i = b0; i < b1; i++) {
          const int u = i - b0;
          const Eigen::Isometry3d T = Tv[ofs[k] + i] * e;
          // ---- 原始点那一路 (只过 range + 动态剔除) ----
          //
          // **一次读盘派生两份**: loadFrame 的实现就是 loadFrameRaw 再加一次 voxelDownsample
          // (前半段的 range 过滤和 removeDynPoints 逐行相同), 所以下面从 pr 拷一份再体素,
          // 结果与直接调 loadFrame 逐位相同, 却省掉一次"读 pcd + lzf 解压 + 读 3dod 标注"。
          // 导出是每帧都要走的, 这一项直接把导出阶段的 IO 减半。
          ialign::PcdCloud pr;
          bool have_raw = false;
          if (raw_mode && (las_ok || rgb_ok)) {
            have_raw = loadFrameRaw(S[k].frames[i].pcd_path, o, pr);
            if (have_raw) {
              if (pr.intensities.size() == pr.points.size()) fir[u] = pr.intensities;
              if (with_rgb) {
                ColorStat cs;
                colorizeFrame(S[k].frames[i].pcd_path, pr.points, ccam, e, o, fcr[u], forr[u], cs);
                // 逐点上色率按**原始点**统计 —— 那才是这两份文件的真实覆盖
                fs_[u].n_pts += cs.n_pts;
                fs_[u].n_col += cs.n_col;
              }
              fpr[u].reserve(pr.points.size());
              for (const auto& p : pr.points) fpr[u].push_back(T * p);
            }
          }
          // ---- 体素那一路 (纯 xyz 的 .pcd, 以及 export_raw=0 时的 las/rgb) ----
          ialign::PcdCloud pc;
          if (have_raw) {
            pc = pr;                       // 上面已经读过盘了, 这里只做体素
            voxelDownsample(pc.points, pc.intensities, o.frame_voxel);
            if (pc.points.empty()) continue;
          } else if (!loadFrame(S[k].frames[i].pcd_path, o, pc)) {
            continue;
          }
          const bool hi = pc.intensities.size() == pc.points.size();
          if (hi) fi[u] = pc.intensities;
          // 体素这一路: 位置和强度总要。颜色只在 export_raw=0 时在这里算 ——
          // raw_mode 下 .las/_rgb.pcd 走原始点, 颜色已经在上面那一路算过了。
          const bool col_here = with_rgb && !raw_mode;
          if (!col_here) {
            fp[u].reserve(pc.points.size());
            for (const auto& p : pc.points) fp[u].push_back(T * p);
            continue;
          }
          std::vector<Rgb> rgb;
          std::vector<char> okc;
          colorizeFrame(S[k].frames[i].pcd_path, pc.points, ccam, e, o, rgb, okc, fs_[u]);
          fp[u].reserve(pc.points.size());
          fc[u].reserve(pc.points.size());
          fo[u].reserve(pc.points.size());
          for (std::size_t q = 0; q < pc.points.size(); q++) {
            fp[u].push_back(T * pc.points[q]);
            fc[u].push_back(rgb[q]);
            fo[u].push_back(okc[q]);
          }
        }
        for (int u = 0; u < b1 - b0; u++) {
          // 原始点 -> 直接写盘
          if (las_ok) lasS.add(fpr[u], &fir[u], with_rgb ? &fcr[u] : nullptr,
                               with_rgb ? &forr[u] : nullptr);
          if (rgb_ok) rgbS.add(fpr[u], nullptr, &fcr[u], &forr[u], o.color_drop != 0);
          fpr[u] = {}; fcr[u] = {}; forr[u] = {}; fir[u] = {};
          // 体素 -> 并进格子
          acc.add(fp[u], (with_rgb && !raw_mode) ? &fc[u] : nullptr,
                  (with_rgb && !raw_mode) ? &fo[u] : nullptr, &fi[u]);
          fp[u] = {}; fc[u] = {}; fo[u] = {}; fi[u] = {};
          tot.n_pts += fs_[u].n_pts;
          tot.n_col += fs_[u].n_col;
        }
      }
      std::vector<Eigen::Vector4d> pts, cpts;
      std::vector<Rgb> cols, cols_all;
      std::vector<double> ints;
      acc.finish(pts, cpts, cols, with_rgb && !raw_mode, o.color_drop != 0, &ints,
                 (o.las && !raw_mode) ? &cols_all : nullptr);
      if (pts.empty()) continue;
      savePts(fs::path(o.out) / (nm + ".pcd"), pts, ints);
      printf("  %-44s %zu 点  [体素 %.2fm, 量重影/厚度的基准]\n", (nm + ".pcd").c_str(),
             pts.size(), o.submap_voxel);
      if (raw_mode) {
        if (las_ok && lasS.finish())
          printf("  %-44s %zu 点  [**原始点**, 绝对 UTM%s]\n", (nm + ".las").c_str(),
                 lasS.size(),
                 o.utm_zone > 0 ? (" EPSG:" + std::to_string(32600 + o.utm_zone)).c_str()
                                : ", 未声明 CRS");
        if (rgb_ok && rgbS.finish())
          printf("  %-44s %zu 点  [**原始点**, 逐点上色率 %.1f%%%s]\n",
                 (nm + "_rgb.pcd").c_str(), rgbS.size(),
                 tot.n_pts ? 100.0 * tot.n_col / tot.n_pts : 0.0,
                 o.color_drop ? ", 未上色的已丢弃 (--color_drop 0 可保留)" : ", 未上色涂深灰");
      } else {
        if (o.las && ialign::saveLas((fs::path(o.out) / (nm + ".las")).string(), pts, ints,
                                     cols_all, base_utm, o.utm_zone))
          printf("  %-44s %zu 点  [体素, 绝对 UTM]\n", (nm + ".las").c_str(), pts.size());
        if (with_rgb) {
          ialign::savePcdRgb((fs::path(o.out) / (nm + "_rgb.pcd")).string(), cpts, cols);
          printf("  %-44s %zu 点  [体素, %.1f%% 的体素有颜色; 逐点上色率 %.1f%%]\n",
                 (nm + "_rgb.pcd").c_str(), cpts.size(),
                 pts.empty() ? 0.0 : 100.0 * cpts.size() / pts.size(),
                 tot.n_pts ? 100.0 * tot.n_col / tot.n_pts : 0.0);
        }
      }
    }
  }
  // ---- 原始点导出 (只导一块方框, 不做体素滤波) ----
  if (o.raw_size > 0) {
    printf("\n=== 原始点导出 (中心 %.0f,%.0f 边长 %.0fm, **不做体素滤波**) ===\n", o.raw_x,
           o.raw_y, o.raw_size);
    const double half = o.raw_size * 0.5;
    for (int k = 0; k < ns; k++) {
      for (int pass = 0; pass < 2; pass++) {
        const bool after = pass == 1;
        if (!after && !o.export_before) continue;
        const auto& Tv = after ? Tcur : Tins;
        const Eigen::Isometry3d& e = after ? ext : Tvl0[k];   // _before 要**原始**外参
        const std::string nm = S[k].name + (after ? "_after" : "_before");
        // 帧的选取要放宽 max_range: 框边上的帧也会往框里打点
        std::vector<int> use;
        for (std::size_t i = 0; i < S[k].frames.size(); i++) {
          const auto t = (Tv[ofs[k] + i] * e).translation();
          if (std::abs(t.x() - o.raw_x) < half + o.max_range &&
              std::abs(t.y() - o.raw_y) < half + o.max_range)
            use.push_back(static_cast<int>(i));
        }
        if (use.empty()) continue;
        std::vector<Eigen::Vector4d> pts;
        std::vector<double> ints;
        std::vector<Rgb> cols;
        std::vector<char> okc_all;
        const bool with_rgb = !ccam.empty();
        for (std::size_t b = 0; b < use.size(); b++) {
          ialign::PcdCloud pc;
          if (!loadFrameRaw(S[k].frames[use[b]].pcd_path, o, pc)) continue;
          const bool hi = pc.intensities.size() == pc.points.size();
          std::vector<Rgb> rgb;
          std::vector<char> okc;
          if (with_rgb) {
            ColorStat cs;
            colorizeFrame(S[k].frames[use[b]].pcd_path, pc.points, ccam, e, o, rgb, okc, cs);
          }
          const Eigen::Isometry3d T = Tv[ofs[k] + use[b]] * e;
          for (std::size_t q = 0; q < pc.points.size(); q++) {
            const Eigen::Vector4d w = T * pc.points[q];
            // 裁到方框: 不裁的话每帧 40m 量程会把框外的东西一起带进来
            if (std::abs(w.x() - o.raw_x) > half || std::abs(w.y() - o.raw_y) > half) continue;
            pts.push_back(w);
            ints.push_back(hi ? pc.intensities[q] : 0.0);
            if (with_rgb) { cols.push_back(rgb[q]); okc_all.push_back(okc[q]); }
          }
        }
        if (pts.empty()) continue;
        savePts(fs::path(o.out) / (nm + "_raw.pcd"), pts, ints);
        printf("  %-46s %zu 点 (%zu 帧)\n", (nm + "_raw.pcd").c_str(), pts.size(), use.size());
        if (o.las) {
          // 未上到色的点给黑, 不给灰占位: las 里颜色和强度并存, 用强度就能看清结构,
          // 灰占位反而在渲染时和真实的浅色路面混在一起
          std::vector<Rgb> ca(pts.size(), Rgb(0, 0, 0));
          for (std::size_t q = 0; q < pts.size() && with_rgb; q++)
            if (okc_all[q]) ca[q] = cols[q];
          if (ialign::saveLas((fs::path(o.out) / (nm + "_raw.las")).string(), pts, ints, ca,
                              base_utm, o.utm_zone))
            printf("  %-46s %zu 点 (绝对 UTM%s)\n", (nm + "_raw.las").c_str(), pts.size(),
                   o.utm_zone > 0 ? (" EPSG:" + std::to_string(32600 + o.utm_zone)).c_str()
                                  : ", 未声明 CRS");
        }
        if (with_rgb) {
          std::vector<Eigen::Vector4d> cp;
          std::vector<Rgb> cc;
          for (std::size_t q = 0; q < pts.size(); q++) {
            if (o.color_drop && !okc_all[q]) continue;
            cp.push_back(pts[q]);
            cc.push_back(cols[q]);
          }
          if (!cp.empty()) {
            ialign::savePcdRgb((fs::path(o.out) / (nm + "_raw_rgb.pcd")).string(), cp, cc);
            printf("  %-46s %zu 点 (%.0f%% 上到色)\n", (nm + "_raw_rgb.pcd").c_str(), cp.size(),
                   100.0 * cp.size() / pts.size());
          }
        }
      }
    }
  }
  return 0;
}

/// @brief 两 session 联合模式 (--joint)。**已被 --incr 取代**, 保留用于两 session 的对照实验。
///
/// 与 runIncr 的区别: 这里 A/B 的位姿从一开始就在同一个参数向量里、且都自由,
/// 没有"前面的 session 固定"这一步, 也没有存档/复用机制 —— 每次都从头配准。
/// 三 session 以上请用 --incr。
static int runJoint(const Opt& o, ialign::SessionData& A, ialign::SessionData& B,
                   const ialign::CloudCovarianceEstimation& ce) {
  const int na = static_cast<int>(A.frames.size()), nb = static_cast<int>(B.frames.size());
  if (na < 5 || nb < 5) {
    std::cerr << "  !! 帧太少 (A=" << na << " B=" << nb << ")\n";
    return 1;
  }
  std::vector<int> iaAll(na), ibAll(nb);
  for (int i = 0; i < na; i++) iaAll[i] = i;
  for (int i = 0; i < nb; i++) ibAll[i] = i;

  // ---- 初值: 车体位姿 (INS) ----
  std::vector<Eigen::Isometry3d> Tv0(na + nb);
  for (int i = 0; i < na; i++) Tv0[i] = A.frames[i].T_w_v;
  for (int i = 0; i < nb; i++) Tv0[na + i] = B.frames[i].T_w_v;
  const Eigen::Isometry3d ext0 = A.T_v_l;

  ialign::SubmapCeresOpts co;
  co.sigma_ins_xy = o.ba_ins_xy;
  co.sigma_rel_t = o.ba_rel_t;
  co.sigma_rel_r = o.ba_rel_r;
  co.attitude_w = o.ba_att_w;
  co.huber = o.ba_huber;
  co.iters = o.ba_iters;
  co.sigma_px = o.ba_sigma_px;
  co.opt_ext = o.opt_ext;
  co.report = true;

  // ---- 阶段 1: 各自整段 BA ----
  printf("\n=== 阶段1: A/B 各自整段 BA ===\n");
  std::vector<ialign::CamModel> vcamA, vcamB;   // 视觉约束用 (上色用的是后面的 camsA/camsB)
  if (o.ba_visual) {
    vcamA = ialign::loadCameras(A.root, o.data_mode, A.T_v_l, o.vis_cams, A.frames[0].pcd_path);
    vcamB = ialign::loadCameras(B.root, o.data_mode, B.T_v_l, o.vis_cams, B.frames[0].pcd_path);
    printf("  [视觉] A 相机 %zu 个, B 相机 %zu 个:", vcamA.size(), vcamB.size());
    for (const auto& c : vcamA) printf(" %s", c.name.c_str());
    printf("\n");
    if (vcamA.empty() || vcamB.empty())
      printf("  [视觉] !! 有 session 一个相机都没读到, 该侧没有视觉约束\n");
  }
  const auto ca = buildIntra(A, iaAll, 0, o, ce, vcamA, "A");
  const auto cb = buildIntra(B, ibAll, na, o, ce, vcamB, "B");

  std::vector<Eigen::Isometry3d> Tv1 = Tv0;
  Eigen::Isometry3d ext1 = ext0;
  {
    // 只放 A 的约束解一次, 再只放 B 的解一次 —— 两者不共享参数, 所以可以分开,
    // 但为了少写一套解算器, 直接放一起解(约束不交叉, 等价于分别解)。
    std::vector<std::tuple<int, int, Eigen::Isometry3d>> rel = ca.rel;
    rel.insert(rel.end(), cb.rel.begin(), cb.rel.end());
    std::vector<ialign::VisReprojSpecExt> vis = ca.vis;
    vis.insert(vis.end(), cb.vis.begin(), cb.vis.end());
    printf("  约束: 帧间 %zu 条, 视觉 %zu 条\n", rel.size(), vis.size());
    const std::vector<int> grp(rel.size(), 0);
    std::vector<Eigen::Isometry3d> out;
    CalibIn ci{&Tv0, &ext0, &rel, &vis, &grp, nullptr};
    const auto st = solveCalib(ci, co, o.calib_rounds, o.cross_sigma_xy, o.cross_sigma_r, out,
                              ext1, "阶段1", o.calib_priors != 0);
    if (!st.ok) {
      std::cerr << "  !! 阶段1 求解失败\n";
      return 1;
    }
    Tv1 = out;
    std::vector<double> da, db;
    for (int i = 0; i < na; i++) da.push_back((Tv1[i].translation() - Tv0[i].translation()).norm());
    for (int i = 0; i < nb; i++) db.push_back((Tv1[na + i].translation() - Tv0[na + i].translation()).norm());
    std::sort(da.begin(), da.end());
    std::sort(db.begin(), db.end());
    printf("  阶段1 位移: A 中位=%.3f p90=%.3f | B 中位=%.3f p90=%.3f m\n", da[da.size() / 2],
           da[da.size() * 9 / 10], db[db.size() / 2], db[db.size() * 9 / 10]);
  }
  const auto Tl = [&](const std::vector<Eigen::Isometry3d>& Tv, const Eigen::Isometry3d& e, int g) {
    return Tv[g] * e;
  };

  // ---- 阶段 2: 遍历重叠区域, 建跨 session 约束 ----
  printf("\n=== 阶段2: 遍历重叠区域建跨 session 约束 ===\n");
  std::vector<std::tuple<int, int, Eigen::Isometry3d>> cross;
  std::vector<double> cross_nn0, cross_nn1;
  int n_region = 0, n_try = 0;
  {
    // 每个 B 帧到最近 A 帧 (用阶段1 之后的位姿)
    std::vector<int> nearA(nb, -1);
    std::vector<double> nearD(nb, 1e18);
#pragma omp parallel for num_threads(o.num_threads) schedule(static)
    for (std::int64_t j = 0; j < nb; j++) {
      const Eigen::Vector2d p = Tl(Tv1, ext1, na + j).translation().head<2>();
      for (int i = 0; i < na; i++) {
        const double d = (Tl(Tv1, ext1, i).translation().head<2>() - p).norm();
        if (d < nearD[j]) { nearD[j] = d; nearA[j] = i; }
      }
    }
    // 沿共位帧按弧长每 region_step 米开一个区域
    std::vector<int> col;
    for (int j = 0; j < nb; j++) {
      if (nearD[j] <= o.b_max_dist) col.push_back(j);
    }
    printf("  共位 B 帧 = %zu / %d (阈值 %.1fm)\n", col.size(), nb, o.b_max_dist);
    double acc = 1e18;
    for (std::size_t t = 0; t < col.size(); t++) {
      if (t > 0) {
        acc += (Tl(Tv1, ext1, na + col[t]).translation().head<2>() -
                Tl(Tv1, ext1, na + col[t - 1]).translation().head<2>()).norm();
      }
      if (acc < o.region_step) continue;
      acc = 0;
      const Eigen::Vector2d ctr = Tl(Tv1, ext1, na + col[t]).translation().head<2>();
      // A 侧: 区域内的帧拼 submap (用阶段1 的位姿)
      std::vector<int> ra;
      for (int i = 0; i < na; i++) {
        if ((Tl(Tv1, ext1, i).translation().head<2>() - ctr).norm() <= o.region_radius) ra.push_back(i);
      }
      // B 侧: 区域内且共位的帧
      std::vector<int> rb;
      for (const int j : col) {
        if ((Tl(Tv1, ext1, na + j).translation().head<2>() - ctr).norm() <= o.region_radius) rb.push_back(j);
      }
      if (ra.size() < 8 || rb.empty()) continue;

      std::vector<Eigen::Vector4d> sm;
      for (const int i : ra) {
        ialign::PcdCloud pc;
        if (!loadFrame(A.frames[i].pcd_path, o, pc)) continue;
        const Eigen::Isometry3d T = Tl(Tv1, ext1, i);
        for (const auto& p : pc.points) sm.push_back(T * p);
      }
      if (sm.size() < 20000) continue;
      { std::vector<double> none; voxelDownsample(sm, none, o.submap_voxel); }
      auto tgt = std::make_shared<gtsam_points::PointCloudCPU>();
      tgt->add_points(sm);
      addCovs(tgt, ce, o.num_threads);
      auto vmm = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.tgt_voxel);
      vmm->insert(*tgt);
      gtsam_points::KdTree tree(tgt->points, tgt->size());
      Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
      for (const auto& p : sm) { mn = mn.cwiseMin(p.head<2>()); mx = mx.cwiseMax(p.head<2>()); }
      const auto gm = ialign::buildGroundMap({{tgt.get(), Eigen::Isometry3d::Identity()}}, mn.x(),
                                            mn.y(), mx.x(), mx.y(), 8.0);
      const auto occ = buildOcc(sm, 2.0);
      n_region++;

      std::vector<Eigen::Isometry3d> res(rb.size());
      std::vector<char> ok(rb.size(), 0);
      std::vector<double> n0(rb.size(), -1), n1(rb.size(), -1);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
      for (std::int64_t q = 0; q < static_cast<std::int64_t>(rb.size()); q++) {
        const int j = rb[q];
        ialign::PcdCloud pc;
        if (!loadFrame(B.frames[j].pcd_path, o, pc)) continue;
        auto src = std::make_shared<gtsam_points::PointCloudCPU>();
        src->add_points(pc.points);
        addCovs(src, ce, 1);
        const Eigen::Isometry3d T0 = Tl(Tv1, ext1, na + j);
        gtsam::Values vals;
        vals.insert(0, gtsam::Pose3(T0.matrix()));
        gtsam::NonlinearFactorGraph g;
        auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vmm, src);
        g.add(f);
        Eigen::Isometry3d T = T0;
        try {
          gtsam_points::LevenbergMarquardtExtParams lm;
          lm.setMaxIterations(o.iters);
          vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
          T = Eigen::Isometry3d(vals.at<gtsam::Pose3>(0).matrix());
        } catch (const std::exception&) {
          continue;
        }
        const double inl = f->inlier_fraction();
        if (o.fine && inl >= o.min_inlier) {
          gtsam::Values v2;
          v2.insert(0, gtsam::Pose3(T.matrix()));
          gtsam::NonlinearFactorGraph g2;
          auto ff = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(
            gtsam::Pose3(), 0, tgt, src,
            std::shared_ptr<gtsam_points::NearestNeighborSearch>(
              &tree, [](gtsam_points::NearestNeighborSearch*) {}));
          ff->set_max_correspondence_distance(o.fine_corr);
          g2.add(ff);
          try {
            gtsam_points::LevenbergMarquardtExtParams lm2;
            lm2.setMaxIterations(o.fine_iters);
            v2 = gtsam_points::LevenbergMarquardtOptimizerExt(g2, v2, lm2).optimize();
            T = Eigen::Isometry3d(v2.at<gtsam::Pose3>(0).matrix());
          } catch (const std::exception&) {
          }
        }
        const Eigen::Isometry3d Ta = o.clamp_planar ? clampPlanar(T, T0) : T;
        n0[q] = aboveGroundNN(*tgt, gm, tree, *src, T0, o.z_above, 3000, 3.0, &occ, 2.0);
        n1[q] = aboveGroundNN(*tgt, gm, tree, *src, Ta, o.z_above, 3000, 3.0, &occ, 2.0);
        const double corr = (Ta.translation().head<2>() - T0.translation().head<2>()).norm();
        if (inl < o.min_inlier || corr > o.max_corr) continue;
        if (n0[q] > 0 && n1[q] > 0 && n1[q] > n0[q]) continue;
        res[q] = Ta;
        ok[q] = 1;
      }
      for (std::size_t q = 0; q < rb.size(); q++) {
        n_try++;
        if (!ok[q]) continue;
        // 约束形式: **A 的最近帧 与 B 帧之间的相对位姿(雷达系)**。
        // 用相对量而不是绝对位姿 —— 绝对位姿会把全局规范一起钉死, 而我们要的是
        // "B 相对 A 摆对", 全局规范由阶段1 的锚定帧和 INS 先验负责。
        const int ai = nearA[rb[q]];
        const Eigen::Isometry3d rel = Tl(Tv1, ext1, ai).inverse() * res[q];
        cross.emplace_back(ai, na + rb[q], rel);
        if (n0[q] > 0) cross_nn0.push_back(n0[q]);
        if (n1[q] > 0) cross_nn1.push_back(n1[q]);
      }
    }
  }
  {
    auto m = [](std::vector<double>& v) {
      if (v.empty()) return -1.0;
      std::sort(v.begin(), v.end());
      return v[v.size() / 2];
    };
    printf("  重叠区域=%d  尝试配准=%d  建成约束=%zu\n"
           "  区域内 nn: 配准前 中位=%.3f -> 配准后 中位=%.3f m\n",
           n_region, n_try, cross.size(), m(cross_nn0), m(cross_nn1));
  }
  if (cross.empty()) {
    std::cerr << "  !! 一条跨 session 约束都没建出来\n";
    return 1;
  }

  // ---- 阶段 3: 联合优化 ----
  printf("\n=== 阶段3: 联合优化 (A内部 + B内部 + 跨session) ===\n");
  std::vector<Eigen::Isometry3d> Tv2 = Tv1;
  Eigen::Isometry3d ext2 = ext1;
  {
    // 同 session 与跨 session 的 sigma **逐条给**, 不再用"重复约束 k 次"那种近似。
    std::vector<std::tuple<int, int, Eigen::Isometry3d>> all = ca.rel;
    all.insert(all.end(), cb.rel.begin(), cb.rel.end());
    std::vector<int> grp(all.size(), 0);
    all.insert(all.end(), cross.begin(), cross.end());
    grp.resize(all.size(), 1);
    std::vector<ialign::VisReprojSpecExt> vis = ca.vis;
    vis.insert(vis.end(), cb.vis.begin(), cb.vis.end());
    printf("  约束: 同session 帧间 %zu 条 (sigma t=%.3f r=%.4f) + 跨session %zu 条"
           " (sigma t=%.3f r=%.4f) + 视觉 %zu 条\n",
           all.size() - cross.size(), o.ba_rel_t, o.ba_rel_r, cross.size(), o.cross_sigma_xy,
           o.cross_sigma_r, vis.size());
    std::vector<Eigen::Isometry3d> out;
    CalibIn ci{&Tv1, &ext1, &all, &vis, &grp, nullptr};
    const auto st = solveCalib(ci, co, o.calib_rounds, o.cross_sigma_xy, o.cross_sigma_r, out,
                              ext2, "阶段3", o.calib_priors != 0);
    if (!st.ok) {
      std::cerr << "  !! 阶段3 求解失败\n";
      return 1;
    }
    Tv2 = out;
    std::vector<double> da, db;
    for (int i = 0; i < na; i++) da.push_back((Tv2[i].translation() - Tv1[i].translation()).norm());
    for (int i = 0; i < nb; i++) db.push_back((Tv2[na + i].translation() - Tv1[na + i].translation()).norm());
    std::sort(da.begin(), da.end());
    std::sort(db.begin(), db.end());
    printf("  阶段3 相对阶段1 的位移: A 中位=%.3f p90=%.3f | B 中位=%.3f p90=%.3f m\n"
           "    A 该很小(它是参照), B 该和跨session 误差同量级\n",
           da[da.size() / 2], da[da.size() * 9 / 10], db[db.size() / 2], db[db.size() * 9 / 10]);
  }

  // ---- 导出 ----
  // **逐份构建并写盘**: 2500 帧 x 3 万点 x 32 字节 = 2.4GB/份, 四份同时攒会打满内存
  // (这个项目已经因为这个被 OOM 看守终止过四次)。
// 导出阶段每帧只顺序读一次, 缓存命中率为零 —— 留着纯粹和体素累积争内存
  // (全量体素那一路要把所有点收进内存, 12000 帧量级是 10 GB)。先报配准阶段的命中率再清空。
  g_fcache.report("配准阶段");
  g_fcache.clear();
  printf("\n=== 导出点云 ===\n");
  struct Job { const char* name; int sofs, cnt; const std::vector<Eigen::Isometry3d>* Tv;
               const Eigen::Isometry3d* ext; const ialign::SessionData* S; };
  const Job jobs[] = {
    {"A_before.pcd", 0, na, &Tv0, &ext0, &A}, {"A_after.pcd", 0, na, &Tv2, &ext2, &A},
    {"B_before.pcd", na, nb, &Tv0, &ext0, &B}, {"B_after.pcd", na, nb, &Tv2, &ext2, &B},
  };
  // 上色用的相机: 每个 session 各读一次 (标定在各自 clip 的 calibration/ 下)。
  // 注意 T_c_l 是在 colorizeFrame 里用 T_v_c 和**该份点云对应的外参**现算的 ——
  // _after 用的是 BA 优化后的 T_v_l, 所以颜色和几何是同一套外参, 不会互相错开。
  std::vector<ialign::CamModel> camsA, camsB;
  if (o.color) {
    camsA = ialign::loadCameras(A.root, o.data_mode, ext0, o.color_cams,
                                A.frames.empty() ? "" : A.frames[0].pcd_path);
    camsB = ialign::loadCameras(B.root, o.data_mode, ext0, o.color_cams,
                                B.frames.empty() ? "" : B.frames[0].pcd_path);
    printf("  [上色] A 相机 %zu 个, B 相机 %zu 个:", camsA.size(), camsB.size());
    for (const auto& c : camsA) printf(" %s", c.name.c_str());
    printf("\n         深度门 z>%.2f  距离<%.0fm  遮挡容差 %.2fm/%dpx\n",
           o.color_z_min, o.color_max_range, o.color_occ_tol, o.color_occ_px);
    if (camsA.empty() || camsB.empty())
      printf("  [上色] !! 有 session 一个相机都没读到, 该份只写不带颜色的点云\n");
  }

  for (const auto& jb : jobs) {
    const auto& cams = (jb.S == &A) ? camsA : camsB;
    const bool do_col = o.color && !cams.empty();
    std::vector<Eigen::Vector4d> pts;
    std::vector<Rgb> cols;
    ColorStat tot;
    if (!do_col) {
      for (int k = 0; k < jb.cnt; k++) {
        ialign::PcdCloud pc;
        if (!loadFrame(jb.S->frames[k].pcd_path, o, pc)) continue;
        const Eigen::Isometry3d T = (*jb.Tv)[jb.sofs + k] * (*jb.ext);
        for (const auto& p : pc.points) pts.push_back(T * p);
      }
    } else {
      // 并行到帧: 每帧要解 7 张 1920x1536 的 jpg, 串行跑一份要好几分钟。
      // 先各帧独立算好, 再串行归并 —— 归并进体素格没法安全地并行。
      std::vector<std::vector<Eigen::Vector4d>> fp(jb.cnt);
      std::vector<std::vector<Rgb>> fc(jb.cnt);
      std::vector<ColorStat> fs_(jb.cnt);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
      for (int k = 0; k < jb.cnt; k++) {
        ialign::PcdCloud pc;
        if (!loadFrame(jb.S->frames[k].pcd_path, o, pc)) continue;
        std::vector<Rgb> rgb;
        std::vector<char> ok;
        colorizeFrame(jb.S->frames[k].pcd_path, pc.points, cams, *jb.ext, o, rgb, ok, fs_[k]);
        const Eigen::Isometry3d T = (*jb.Tv)[jb.sofs + k] * (*jb.ext);
        fp[k].reserve(pc.points.size());
        fc[k].reserve(pc.points.size());
        for (std::size_t i = 0; i < pc.points.size(); i++) {
          if (o.color_drop && !ok[i]) continue;
          fp[k].push_back(T * pc.points[i]);
          fc[k].push_back(rgb[i]);
        }
      }
      for (int k = 0; k < jb.cnt; k++) {
        pts.insert(pts.end(), fp[k].begin(), fp[k].end());
        cols.insert(cols.end(), fc[k].begin(), fc[k].end());
        fp[k] = {};
        fc[k] = {};
        tot.n_pts += fs_[k].n_pts;
        tot.n_col += fs_[k].n_col;
        tot.n_occ += fs_[k].n_occ;
        tot.n_img_miss += fs_[k].n_img_miss;
        tot.n_frame_noimg += fs_[k].n_frame_noimg;
      }
    }
    if (pts.empty()) continue;
    if (do_col) {
      voxelDownsampleRgb(pts, cols, o.submap_voxel);
      std::string nm = jb.name;
      nm.replace(nm.size() - 4, 4, "_rgb.pcd");
      ialign::savePcdRgb((fs::path(o.out) / nm).string(), pts, cols);
      printf("  %-18s %zu 点  上到色 %.1f%%  (遮挡挡掉 %ld 次, 缺图 %ld 张",
             nm.c_str(), pts.size(),
             tot.n_pts ? 100.0 * tot.n_col / tot.n_pts : 0.0, tot.n_occ, tot.n_img_miss);
      if (tot.n_frame_noimg) printf(", **%ld 帧一张图都没有**", tot.n_frame_noimg);
      printf(")\n");
      // 不带颜色的那份照旧也写: 量重影的 pcd_diff 只吃 xyz。
      savePts(fs::path(o.out) / jb.name, pts);
    } else {
      std::vector<double> none;
      voxelDownsample(pts, none, o.submap_voxel);
      savePts(fs::path(o.out) / jb.name, pts);
      printf("  %-14s %zu 点\n", jb.name, pts.size());
    }
  }
  {
    std::ofstream f((fs::path(o.out) / "poses_joint.csv").string());
    f << "session,idx,x_ins,y_ins,z_ins,x_stage1,y_stage1,z_stage1,x_joint,y_joint,z_joint\n";
    for (int g = 0; g < na + nb; g++) {
      const auto t0 = (Tv0[g] * ext0).translation();
      const auto t1 = (Tv1[g] * ext1).translation();
      const auto t2 = (Tv2[g] * ext2).translation();
      f << (g < na ? 'A' : 'B') << ',' << (g < na ? g : g - na) << ',' << t0.x() << ',' << t0.y()
        << ',' << t0.z() << ',' << t1.x() << ',' << t1.y() << ',' << t1.z() << ',' << t2.x() << ','
        << t2.y() << ',' << t2.z() << '\n';
    }
  }
  printf("\n  怎么比:\n"
         "    ./build/pcd_diff %s/A_after.pcd %s/B_after.pcd --mutual 2.0 --structured 0.05 --cell_report 10\n"
         "    和 A_before vs B_before 比一次就是优化前后的跨 session 一致性。\n"
         "    poses_joint.csv 里有每帧 INS / 阶段1 / 联合 三个位姿, 拿它看各阶段挪了多少。\n",
         o.out.c_str(), o.out.c_str());
  return 0;
}

// -----------------------------------------------------------------------------
/// @brief 入口。三种模式:
///   --incr          增量建图 (N 个 session 依次并入) -> runIncr
///   --joint         两 session 联合 (旧路径, 对照用)  -> runJoint
///   (默认)          单锚点: 指定 --center, A 拼 submap / B 逐帧配上去
///   --scan_overlap  只扫两条轨迹的共位情况然后退出 (选 --center 之前先跑这个)
///
/// 无论哪种模式, 加载后的处理顺序是固定的, **不能调换**:
///   1) 统一世界系 (各 session 的 utm_center 不同, 不统一的话按距离选邻域会选到别处)
///   2) --load_roi 范围过滤 (必须在 1) 之后, 否则同一个方框框到的是不同地方)
///   3) --yaw_fix_deg 外参 yaw 修正
///   4) 断连/静止块过滤 (dropIsolated)
int main(int argc, char** argv) {
  // 行缓冲: 长时间任务的日志要能实时 tail, 否则重定向到文件时会攒几 KB 才落盘
  setvbuf(stdout, nullptr, _IOLBF, 0);
  Opt o;
  // **先** yaml, **后**命令行 —— 命令行是覆盖。预扫一遍找 --params, 因为 parse() 里
  // 处理它的时候其余选项可能已经被应用过了, 顺序会反。
  for (int i = 1; i + 1 < argc; i++) {
    if (std::string(argv[i]) == "--params") {
      if (!loadParams(argv[i + 1], o)) return 1;
      break;
    }
  }
  if (!parse(argc, argv, o)) return 0;
  g_fcache.setCapacity(static_cast<std::size_t>(std::max(0, o.frame_cache_mb)) * 1048576ULL);
  if (o.frame_cache_mb > 0) {
    printf("帧缓存: 上限 %d MB (--frame_cache_mb 0 可关闭)\n", o.frame_cache_mb);
  }
  if (!o.save_params.empty()) {
    if (!saveParams(o.save_params, o, "local_align 参数模板")) return 1;
    printf("参数已写出: %s\n", o.save_params.c_str());
    return 0;
  }
  fs::create_directories(o.out);
  fs::create_directories(fs::path(o.out) / "frames");

  // target 体素图 vs 点间距的自检。违反这条会让 inlier 静默塌到 0.2 附近,
  // 于是所有帧都过不了 min_inlier —— 而日志里看不出任何异常。
  {
    const double spacing = std::max(o.frame_voxel, o.submap_voxel);
    if (o.tgt_voxel < spacing * 3.0 - 1e-9) {
      printf("!! tgt_voxel=%.2f 相对点间距 %.2f (=max(frame_voxel %.2f, submap_voxel %.2f)) 太小,\n"
             "   建议 >= %.2f。体素图比点间距还稀时, source 点必须落进与 target 点同一个格子\n"
             "   才算 inlier -> 无论对齐多好 inlier 都会塌到 0.2 量级, 帧会全部被剔。\n",
             o.tgt_voxel, spacing, o.frame_voxel, o.submap_voxel, spacing * 3.0);
    }
  }

  printf("=== 配置 ===\n"
         "  中心=(%.1f, %.1f)  半径: A=%.0fm B=%.0fm   A=session%d  B=session%d\n"
         "  data_mode=%s pose_dir=%s attitude=%s lidar=%s\n"
         "  frame_voxel=%.2f submap_voxel=%.2f tgt_voxel=%.2f  range=[%.1f,%.1f]\n"
         "  配准: VGICP %d 迭代%s  inlier>=%.2f  修正上限 %.1fm  平面压制=%s  动态剔除=%s\n",
         o.cx, o.cy, o.radius, o.b_radius, o.sa, o.sb, o.data_mode.c_str(), o.pose_dir.c_str(),
         o.attitude_from.c_str(), o.lidar_dir.c_str(), o.frame_voxel, o.submap_voxel, o.tgt_voxel,
         o.min_range, o.max_range, o.iters,
         o.fine ? (" + GICP 精化 " + std::to_string(o.fine_iters) + " 迭代").c_str() : "",
         o.min_inlier, o.max_corr, o.clamp_planar ? "开" : "关", o.dyn_filter ? "开" : "关");

  // 原来这里有一句 glim::GlobalConfig::instance(o.config) —— 已删。
  // 实测 CloudCovarianceEstimation 的构造只设 regularization_method=PLANE 和线程数,
  // **不读任何配置**; 那句是为 glim 的 GlobalMapping/IMUIntegration 准备的, 本程序两个都不用。
  // 于是 config/ 目录(logging/sensors/global_mapping 那几个 json)整个不再需要。
  ialign::CloudCovarianceEstimation ce(o.num_threads);

  // ---- 加载两个 session ----
  ialign::LoadOptions lo;
  lo.pose_dir = o.pose_dir;
  lo.attitude_from = o.attitude_from;
  lo.data_mode = o.data_mode;
  lo.pose_src = o.pose_src;
  lo.lidar_dir = o.lidar_dir;
  const auto dirs = ialign::discoverSessions(o.root, lo.pose_dir, o.data_mode);
  // sa/sb 只有单锚点/--joint 模式用; --incr 走 sess_order, 单个 session 也合法
  if (!o.incr && (o.sa < 0 || o.sb < 0 || o.sa >= static_cast<int>(dirs.size()) ||
                  o.sb >= static_cast<int>(dirs.size()))) {
    std::cerr << "session 下标越界: 只找到 " << dirs.size() << " 个\n";
    return 1;
  }
  if (o.incr && dirs.empty()) {
    std::cerr << "没找到任何 session (root=" << o.root << ")\n";
    return 1;
  }
  // ---- 增量模式: 加载全部 session, 统一到第一个的世界系, 然后依次并入 ----
  if (o.incr) {
    std::vector<int> order;
    if (o.sess_order.empty()) {
      for (int i = 0; i < static_cast<int>(dirs.size()); i++) order.push_back(i);
    } else {
      std::size_t p = 0;
      while (p <= o.sess_order.size()) {
        const auto q = o.sess_order.find(',', p);
        const std::string t =
          o.sess_order.substr(p, q == std::string::npos ? std::string::npos : q - p);
        if (!t.empty()) {
          int hit = -1;
          for (int i = 0; i < static_cast<int>(dirs.size()); i++) {
            if (dirs[i].filename().string() == t) { hit = i; break; }
          }
          if (hit < 0) {
            std::cerr << "--sess_order 里的 " << t << " 在 " << o.root << " 下找不到\n";
            return 1;
          }
          order.push_back(hit);
        }
        if (q == std::string::npos) break;
        p = q + 1;
      }
    }
    std::vector<ialign::SessionData> S(order.size());
    for (std::size_t k = 0; k < order.size(); k++) {
      if (!ialign::loadSession(dirs[order[k]], order[k], S[k], lo)) {
        std::cerr << "加载 session 失败: " << dirs[order[k]] << "\n";
        return 1;
      }
    }
    if (S.empty()) {
      std::cerr << "一个 session 都没加载到\n";
      return 1;
    }
    // 统一世界系到第一个 session 的 utm_center
    const Eigen::Vector2d base_utm = S[0].utm_center;
    printf("\n  基准 session = %s (utm_center %.2f, %.2f)\n", S[0].name.c_str(), base_utm.x(),
           base_utm.y());
    for (std::size_t k = 1; k < S.size(); k++) {
      const Eigen::Vector2d d2 = S[k].utm_center - base_utm;
      if (d2.norm() > 1e-6) {
        const Eigen::Vector3d d3(d2.x(), d2.y(), 0.0);
        for (auto& f : S[k].frames) {
          f.T_w_v.translation() += d3;
          f.T_w_l.translation() += d3;
        }
        S[k].utm_center = base_utm;
        printf("  %s 的 utm_center 差 (%.2f, %.2f)m, 已平移到基准系\n", S[k].name.c_str(), d2.x(),
               d2.y());
      }
    }
    for (const auto& sd : S) {
      if (sd.frames.empty()) continue;
      Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
      for (const auto& f : sd.frames) {
        mn = mn.cwiseMin(f.T_w_l.translation().head<2>());
        mx = mx.cwiseMax(f.T_w_l.translation().head<2>());
      }
      printf("  %-34s 帧=%-5zu 包围盒 x[%.0f, %.0f] y[%.0f, %.0f]\n", sd.name.c_str(),
             sd.frames.size(), mn.x(), mx.x(), mn.y(), mx.y());
    }
    if (o.load_roi_size > 0.0) {
      const double half = o.load_roi_size * 0.5;
      printf("  加载期范围过滤: 中心(%.1f, %.1f) 边长 %.0fm\n", o.load_roi_x, o.load_roi_y,
             o.load_roi_size);
      for (auto& sd : S) {
        const std::size_t before = sd.frames.size();
        std::vector<ialign::Frame> keep;
        for (const auto& f : sd.frames) {
          const auto& t = f.T_w_l.translation();
          if (std::abs(t.x() - o.load_roi_x) <= half && std::abs(t.y() - o.load_roi_y) <= half)
            keep.push_back(f);
        }
        sd.frames.swap(keep);
        printf("    %-34s %zu -> %zu 帧\n", sd.name.c_str(), before, sd.frames.size());
      }
    }
    if (std::abs(o.yaw_fix_deg) > 1e-9) {
      const Eigen::Isometry3d Rz(
        Eigen::AngleAxisd(o.yaw_fix_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
      for (auto& sd : S) {
        for (auto& f : sd.frames) f.T_w_l = f.T_w_l * Rz;
      }
      printf("  外参 yaw 修正: %+.3f 度\n", o.yaw_fix_deg);
    }
    {
      std::vector<ialign::SessionData*> ps;
      for (auto& sd : S) ps.push_back(&sd);
      dropIsolated(ps, o.drop_isolated, o.drop_static, o.drop_frames != 0);
    }
    fs::create_directories(o.out);
    return runIncr(o, S, base_utm, ce);
  }

  ialign::SessionData A, B;
  if (!ialign::loadSession(dirs[o.sa], o.sa, A, lo) || !ialign::loadSession(dirs[o.sb], o.sb, B, lo)) {
    std::cerr << "加载 session 失败\n";
    return 1;
  }
  printf("\n  A = session%d %s  帧=%zu\n  B = session%d %s  帧=%zu\n", o.sa, A.name.c_str(),
         A.frames.size(), o.sb, B.name.c_str(), B.frames.size());

  // ---- 统一世界系 ----
  // 各 session 的位姿是"相对自己 utm_center"的局部坐标, centers 不同就不在同一系里,
  // 按距离选邻域会选到完全不同的地方。都是 UTM 投影系, 只差基准平移, 加差值即可精确统一。
  {
    const Eigen::Vector2d d2 = B.utm_center - A.utm_center;
    if (d2.norm() > 1e-6) {
      const Eigen::Vector3d d3(d2.x(), d2.y(), 0.0);
      for (auto& f : B.frames) {
        f.T_w_v.translation() += d3;
        f.T_w_l.translation() += d3;
      }
      B.utm_center = A.utm_center;
      printf("  B 的 utm_center 与 A 差 (%.2f, %.2f)m, 已平移到 A 的系\n", d2.x(), d2.y());
    }
  }
  for (const auto* s : {&A, &B}) {
    if (s->frames.empty()) continue;
    Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
    for (const auto& f : s->frames) {
      mn = mn.cwiseMin(f.T_w_l.translation().head<2>());
      mx = mx.cwiseMax(f.T_w_l.translation().head<2>());
    }
    printf("  %s 轨迹包围盒: x[%.0f, %.0f] y[%.0f, %.0f]\n", s->name.c_str(), mn.x(), mx.x(), mn.y(),
           mx.y());
  }

  // ---- 加载期范围过滤 (联合模式先在小范围试跑) ----
  // 必须放在统一世界系**之后** —— 否则两个 session 的坐标不在同一系里, 同一个方框
  // 选出的是不同地方的数据。
  if (o.load_roi_size > 0.0) {
    const double half = o.load_roi_size * 0.5;
    printf("  加载期范围过滤: 中心(%.1f, %.1f) 边长 %.0fm\n", o.load_roi_x, o.load_roi_y,
           o.load_roi_size);
    for (auto* sd : {&A, &B}) {
      const std::size_t before = sd->frames.size();
      std::vector<ialign::Frame> keep;
      for (const auto& f : sd->frames) {
        const auto& t = f.T_w_l.translation();
        if (std::abs(t.x() - o.load_roi_x) <= half && std::abs(t.y() - o.load_roi_y) <= half) {
          keep.push_back(f);
        }
      }
      sd->frames.swap(keep);
      printf("    %s: %zu -> %zu 帧\n", sd->name.c_str(), before, sd->frames.size());
    }
  }

  // ---- 给外参补 yaw 修正 ----
  // 右乘: T_w_l' = T_w_l * Rz(delta) —— 修的是**雷达系相对车体的朝向**, 所以在雷达侧乘。
  // 两个 session 用同一套车/标定, 所以都要修。
  if (std::abs(o.yaw_fix_deg) > 1e-9) {
    const Eigen::Isometry3d Rz(Eigen::AngleAxisd(o.yaw_fix_deg * M_PI / 180.0,
                                                Eigen::Vector3d::UnitZ()));
    for (auto* sd : {&A, &B}) {
      for (auto& f : sd->frames) f.T_w_l = f.T_w_l * Rz;
    }
    printf("  外参 yaw 修正: %+.3f 度 (右乘到 T_w_l)\n", o.yaw_fix_deg);
  }
  dropIsolated({&A, &B}, o.drop_isolated, o.drop_static, o.drop_frames != 0);

  // ---- 联合模式 ----
  if (o.joint) return runJoint(o, A, B, ce);

  // ---- 只扫共位情况 ----
  if (o.scan_overlap) {
    const auto yawOf = [](const Eigen::Isometry3d& T) {
      Eigen::Vector3d f = T.linear().col(0);
      f.z() = 0;
      return f.norm() > 1e-6 ? std::atan2(f.y(), f.x()) : 0.0;
    };
    struct Hit { int bj; int ai; double d, dyaw; };
    std::vector<Hit> hits(B.frames.size());
#pragma omp parallel for num_threads(o.num_threads) schedule(static)
    for (std::int64_t j = 0; j < static_cast<std::int64_t>(B.frames.size()); j++) {
      double bd = 1e18;
      int bi = -1;
      for (std::size_t i = 0; i < A.frames.size(); i++) {
        const double d = (A.frames[i].T_w_l.translation().head<2>() -
                          B.frames[j].T_w_l.translation().head<2>()).norm();
        if (d < bd) { bd = d; bi = static_cast<int>(i); }
      }
      double da = 0;
      if (bi >= 0) {
        da = std::abs(yawOf(A.frames[bi].T_w_l) - yawOf(B.frames[j].T_w_l));
        while (da > M_PI) da = 2 * M_PI - da;
        da *= 180.0 / M_PI;
      }
      hits[j] = {static_cast<int>(j), bi, bd, da};
    }
    std::vector<double> dv;
    for (const auto& h : hits) dv.push_back(h.d);
    std::sort(dv.begin(), dv.end());
    const auto q = [&](double f) {
      return dv.empty() ? -1.0 : dv[std::min(dv.size() - 1, static_cast<std::size_t>(f * dv.size()))];
    };
    printf("\n=== 两条轨迹的共位情况 (每个 B 帧到最近 A 帧) ===\n"
           "  距离: 中位=%.1f p10=%.1f p25=%.1f p50=%.1f p90=%.1f min=%.2f max=%.1f m\n",
           q(0.5), q(0.1), q(0.25), q(0.5), q(0.9), dv.empty() ? -1.0 : dv.front(),
           dv.empty() ? -1.0 : dv.back());
    for (const double thr : {2.0, 5.0, 10.0, 20.0}) {
      std::size_t n = 0;
      for (const auto& h : hits) {
        if (h.d <= thr) n++;
      }
      printf("  距离 <= %5.1fm 的 B 帧: %zu / %zu (%.1f%%)\n", thr, n, hits.size(),
             100.0 * n / std::max<std::size_t>(1, hits.size()));
    }
    // 连续共位段: 拿它选 --center
    printf("\n  共位段 (连续的 B 帧, 距离<=%.1fm; 拿中心坐标去做 --center):\n", o.b_max_dist);
    std::size_t st = 0;
    bool in = false;
    int nseg = 0;
    for (std::size_t j = 0; j <= hits.size(); j++) {
      const bool ok = j < hits.size() && hits[j].d <= o.b_max_dist;
      if (ok && !in) { st = j; in = true; }
      if (!ok && in) {
        in = false;
        const std::size_t n = j - st;
        if (n < 5) continue;   // 太短的段不值得测
        Eigen::Vector2d c(0, 0);
        double dsum = 0, asum = 0;
        for (std::size_t k = st; k < j; k++) {
          c += B.frames[hits[k].bj].T_w_l.translation().head<2>();
          dsum += hits[k].d;
          asum += hits[k].dyaw;
        }
        c /= static_cast<double>(n);
        printf("    段%-2d  %3zu 帧  中心=(%.0f, %.0f)  距离均值=%.2fm  航向差均值=%.0f度"
               "   --center %.0f,%.0f\n",
               ++nseg, n, c.x(), c.y(), dsum / n, asum / n, c.x(), c.y());
      }
    }
    if (nseg == 0) {
      printf("    (没有长度 >=5 帧的共位段。把 --b_max_dist 放大再看, 或者这两趟确实没走同一条路)\n");
    }

    // ---- 候选中心: 沿共位帧每隔 scan_step 米取一个, 直接可拿去 --center ----
    // 上面那个"段"是按帧号连续性切的, 一段可能横跨几公里 —— 它的均值中心没法用来定位。
    // 这里按**弧长**采样, 并报出每处的局部情况(A/B 帧数、到最近 A 帧的距离), 因为
    // "有共位帧"不等于"够拼一个 submap": A 帧太少时 submap 本身就立不起来。
    printf("\n  候选中心 (沿共位帧每 %.0fm 一个; A/B 帧数按 --radius %.0fm 统计):\n",
           o.scan_step, o.radius);
    {
      std::vector<int> col;
      for (const auto& h : hits) {
        if (h.d <= o.b_max_dist) col.push_back(h.bj);
      }
      double acc = 1e18;   // 让第一个就被采纳
      int ncand = 0;
      for (std::size_t t = 0; t < col.size(); t++) {
        if (t > 0) {
          acc += (B.frames[col[t]].T_w_l.translation().head<2>() -
                  B.frames[col[t - 1]].T_w_l.translation().head<2>()).norm();
        }
        if (acc < o.scan_step) continue;
        acc = 0;
        const Eigen::Vector2d c = B.frames[col[t]].T_w_l.translation().head<2>();
        std::size_t nA = 0, nB = 0;
        double dsum = 0;
        for (const auto& f : A.frames) {
          if ((f.T_w_l.translation().head<2>() - c).norm() <= o.radius) nA++;
        }
        for (const auto& h : hits) {
          if (h.d > o.b_max_dist) continue;
          if ((B.frames[h.bj].T_w_l.translation().head<2>() - c).norm() > o.radius) continue;
          nB++;
          dsum += h.d;
        }
        if (nA < 8 || nB < 4) continue;   // 拼不出 submap / 没帧可配, 不值得跑
        printf("    候选%-2d  A=%3zu 帧  B(共位)=%3zu 帧  到最近A均值=%.2fm"
               "   --center %.0f,%.0f\n",
               ++ncand, nA, nB, nB ? dsum / nB : -1.0, c.x(), c.y());
      }
      if (ncand == 0) {
        printf("    (没有 A>=8 且 B>=4 帧的位置。把 --radius 放大, 或这两趟重合得太少)\n");
      }
    }
    printf("\n  判读: 距离大 + 航向差接近 90/180 度 = 两趟只是穿过同一片区域, 没走同一条路。\n"
           "        那里量出来的\"错位\"是视角差异(遮挡/可见面/采样密度都不同), 配准修不了。\n");
    return 0;
  }

  // ---- 选邻域内的帧 ----
  const Eigen::Vector2d ctr(o.cx, o.cy);
  std::vector<int> ia, ib;
  for (std::size_t i = 0; i < A.frames.size(); i++) {
    if ((A.frames[i].T_w_l.translation().head<2>() - ctr).norm() <= o.radius) ia.push_back(static_cast<int>(i));
  }
  for (std::size_t i = 0; i < B.frames.size(); i++) {
    if ((B.frames[i].T_w_l.translation().head<2>() - ctr).norm() <= o.b_radius) ib.push_back(static_cast<int>(i));
  }
  printf("\n  邻域内: A=%zu 帧  B=%zu 帧\n", ia.size(), ib.size());
  if (ia.size() < 2 || ib.empty()) {
    std::cerr << "  !! 邻域内帧太少, 换个中心或加大半径 (照上面的包围盒选)\n";
    return 1;
  }

  // ---- 按与 A 关键帧的位姿关系筛 B 帧 ----
  // **先把分布打出来再筛** —— 这个项目里已经三次因为"用错量级的参照物定阈值"而把
  // 全部候选静默剔掉, 所以阈值必须对着实测分布定, 不能反过来。
  {
    const auto yawOf = [](const Eigen::Isometry3d& T) {
      Eigen::Vector3d f = T.linear().col(0);
      f.z() = 0;
      return f.norm() > 1e-6 ? std::atan2(f.y(), f.x()) : 0.0;
    };
    std::vector<double> dv, av;
    std::vector<int> keep;
    std::size_t drop_d = 0, drop_a = 0;
    for (const int j : ib) {
      double bd = 1e18, ba = 0;
      for (const int i : ia) {
        const double d =
          (A.frames[i].T_w_l.translation().head<2>() - B.frames[j].T_w_l.translation().head<2>()).norm();
        if (d < bd) {
          bd = d;
          double da = std::abs(yawOf(A.frames[i].T_w_l) - yawOf(B.frames[j].T_w_l));
          while (da > M_PI) da = 2 * M_PI - da;
          ba = da * 180.0 / M_PI;
        }
      }
      dv.push_back(bd);
      av.push_back(ba);
      if (o.b_max_dist > 0 && bd > o.b_max_dist) { drop_d++; continue; }
      if (o.b_max_dyaw < 180 && ba > o.b_max_dyaw) { drop_a++; continue; }
      keep.push_back(j);
    }
    std::vector<double> ds = dv, as = av;
    std::sort(ds.begin(), ds.end());
    std::sort(as.begin(), as.end());
    const auto q = [](const std::vector<double>& v, double f) {
      return v.empty() ? -1.0 : v[std::min(v.size() - 1, static_cast<std::size_t>(f * v.size()))];
    };
    printf("  B 帧到最近 A 关键帧: 距离 中位=%.2f p10=%.2f p90=%.2f max=%.2f m\n"
           "                        航向差 中位=%.1f p90=%.1f max=%.1f 度\n"
           "  按 dist<=%.1fm / dyaw<=%.0f度 筛: 保留 %zu 帧 (距离剔=%zu 航向剔=%zu)\n",
           q(ds, 0.5), q(ds, 0.1), q(ds, 0.9), ds.empty() ? -1.0 : ds.back(), q(as, 0.5),
           q(as, 0.9), as.empty() ? -1.0 : as.back(), o.b_max_dist, o.b_max_dyaw, keep.size(),
           drop_d, drop_a);
    if (keep.empty()) {
      printf("  !! 一帧都没留下。照上面的**实测分布**调 --b_max_dist / --b_max_dyaw,\n"
             "     别照直觉给 —— 若距离中位就已经大于阈值, 说明两条轨迹在这一带本来就没重合。\n");
      return 1;
    }
    ib.swap(keep);
  }

  // ---- 载入 A 的帧 ----
  const auto t0 = std::chrono::steady_clock::now();
  const int na = static_cast<int>(ia.size());
  std::vector<ialign::PcdCloud> apc(na);
  long dyn_sum = 0, dyn_frames = 0, dyn_nomatch = 0;
  {
    std::vector<long> nds(na, -1);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t k = 0; k < na; k++) {
      loadFrame(A.frames[ia[k]].pcd_path, o, apc[k], &nds[k]);
    }
    for (const long nd : nds) {
      if (nd > 0) { dyn_sum += nd; dyn_frames++; } else if (nd < 0) dyn_nomatch++;
    }
  }

  // A 的帧位姿: 初值取 INS, 若开 BA 则被优化覆盖。
  // 全程在**世界系**里做 (submap 局部系只是平移+旋转的重参数化, 没必要多一层变换),
  // 锚定帧取中心帧 —— 它离邻域中心最近, 局部约束最充分。
  std::vector<Eigen::Isometry3d> Ta_w(na), Ta_w0(na);
  for (int k = 0; k < na; k++) Ta_w[k] = Ta_w0[k] = A.frames[ia[k]].T_w_l;
  int a_center = 0;
  {
    double bd = 1e18;
    for (int k = 0; k < na; k++) {
      const double d = (Ta_w0[k].translation().head<2>() - ctr).norm();
      if (d < bd) { bd = d; a_center = k; }
    }
  }

  if (o.ba && na >= 3) {
    printf("\n=== submap 内部 BA (Ceres) ===\n");
    // 帧间 VGICP: 只在重叠够的帧对之间建, 测量是配准出来的相对位姿
    std::vector<gtsam_points::PointCloudCPU::Ptr> ac(na);
    std::vector<gtsam_points::GaussianVoxelMap::Ptr> avm(na);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
    for (std::int64_t k = 0; k < na; k++) {
      if (apc[k].points.empty()) continue;
      auto c = std::make_shared<gtsam_points::PointCloudCPU>();
      c->add_points(apc[k].points);
      addCovs(c, ce, 1);
      auto vmk = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.ba_pair_voxel);
      vmk->insert(*c);
      ac[k] = c;
      avm[k] = vmk;
    }
    std::vector<std::pair<int, int>> cand;
    for (int i = 0; i < na; i++) {
      if (!avm[i]) continue;
      for (int j = i + 1; j < na; j++) {
        if (!ac[j]) continue;
        if (gtsam_points::overlap_auto(avm[i], ac[j], Ta_w0[i].inverse() * Ta_w0[j]) <
            o.ba_min_overlap)
          continue;
        cand.emplace_back(i, j);
      }
    }
    std::vector<std::tuple<int, int, Eigen::Isometry3d>> rel;
    std::vector<double> inls;
    std::size_t n_big = 0;
    {
      std::vector<Eigen::Isometry3d> rr(cand.size());
      std::vector<char> ok(cand.size(), 0);
      std::vector<double> iv(cand.size(), -1);
#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
      for (std::int64_t c = 0; c < static_cast<std::int64_t>(cand.size()); c++) {
        const int i = cand[c].first, j = cand[c].second;
        const Eigen::Isometry3d T0 = Ta_w0[i].inverse() * Ta_w0[j];
        gtsam::Values vals;
        vals.insert(0, gtsam::Pose3(T0.matrix()));
        gtsam::NonlinearFactorGraph g;
        auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, avm[i],
                                                                        ac[j]);
        g.add(f);
        try {
          gtsam_points::LevenbergMarquardtExtParams lm;
          lm.setMaxIterations(o.ba_pair_iters);
          vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
        } catch (const std::exception&) {
          continue;
        }
        const Eigen::Isometry3d T1(vals.at<gtsam::Pose3>(0).matrix());
        iv[c] = f->inlier_fraction();
        // 测量的修正量太大 = 大概率配错了, 宁可不要这条约束
        if ((T1.translation() - T0.translation()).norm() > o.ba_max_corr) continue;
        rr[c] = T1;
        ok[c] = 1;
      }
      for (std::size_t c = 0; c < cand.size(); c++) {
        if (iv[c] >= 0) inls.push_back(iv[c]);
        if (ok[c]) rel.emplace_back(cand[c].first, cand[c].second, rr[c]);
        else if (iv[c] >= 0) n_big++;
      }
    }
    std::sort(inls.begin(), inls.end());
    printf("  帧间 GICP: 候选=%zu 采纳=%zu (修正过大剔=%zu)  inlier 中位=%.3f\n", cand.size(),
           rel.size(), n_big, inls.empty() ? -1.0 : inls[inls.size() / 2]);

    // ---- 视觉重投影对应 ----
    std::vector<ialign::VisReprojSpec> vspec;
    std::vector<ialign::VisualCorr> vcorr;
    std::vector<ialign::CamModel> cams;
    if (o.ba_visual) {
      cams = ialign::loadCameras(A.root, o.data_mode, A.T_v_l, o.vis_cams,
                                 A.frames[ia[0]].pcd_path);
      if (cams.empty()) {
        printf("  [visual] 一个相机都没加载到, 跳过视觉约束\n");
      } else {
        printf("  [visual] 相机 %zu 个:", cams.size());
        for (const auto& c : cams) printf(" %s", c.name.c_str());
        printf("\n");
        ialign::VisualOpts vo;
        vo.enable = true;
        vo.sigma_px = o.ba_sigma_px;
        vo.dump_max = 0;
        const auto img_of = [&](int k, int cami) {
          return ialign::imagePathFor(A.frames[ia[k]].pcd_path, cams[cami].name, o.data_mode);
        };
        const auto cloud_of = [&](int k) {
          return (k >= 0 && k < na) ? apc[k].points : std::vector<Eigen::Vector4d>();
        };
        const auto pcd_of = [&](int k) {
          return (k >= 0 && k < na) ? A.frames[ia[k]].pcd_path : std::string();
        };
        ialign::VisualStats vst;
        vcorr = ialign::buildVisualCorrs(cand, Ta_w0, Ta_w0, cams, img_of, cloud_of, pcd_of,
                                        o.data_mode, A.T_v_l, vo, o.num_threads, vst,
                                        fs::path(o.out) / "visual_debug");
        printf("  [visual] 边=%ld 原始匹配=%ld -> 描述子剔=%ld 极几何剔=%ld 无深度剔=%ld"
               " gate剔=%ld -> **保留=%ld**  建边时误差均值=%.2f px\n",
               vst.edges, vst.raw, vst.drop_desc, vst.drop_fmat, vst.drop_nodepth, vst.drop_gate,
               vst.kept, vst.kept ? vst.err_sum / vst.kept : -1.0);
        vspec.reserve(vcorr.size());
        for (const auto& vc : vcorr) {
          ialign::VisReprojSpec vs;
          vs.i = vc.i;
          vs.j = vc.j;
          vs.p_i = vc.p_i;
          vs.T_c_l = cams[vc.cam].T_c_l.matrix();
          vs.fx = cams[vc.cam].fx;
          vs.fy = cams[vc.cam].fy;
          vs.cx = cams[vc.cam].cx;
          vs.cy = cams[vc.cam].cy;
          vs.u = vc.u;
          vs.v = vc.v;
          vspec.push_back(vs);
        }
      }
    }

    // ---- Ceres ----
    // 重力"上"在世界系下就是 +Z (位姿已在 UTM 局部系, 没有额外旋转)
    ialign::SubmapCeresOpts co;
    co.opt_ext = o.opt_ext;
    co.sigma_ins_xy = o.ba_ins_xy;
    co.sigma_rel_t = o.ba_rel_t;
    co.sigma_rel_r = o.ba_rel_r;
    co.attitude_w = o.ba_att_w;
    co.huber = o.ba_huber;
    co.chi2_reject = o.ba_chi2;
    co.iters = o.ba_iters;
    co.sigma_px = o.ba_sigma_px;
    std::vector<Eigen::Isometry3d> Tout;
    ialign::SubmapCeresStats cst;
    if (o.opt_ext > 0) {
      // 待优化量: 车体位姿 + 共享外参。rel 的测量本来就是**雷达系之间**的相对位姿,
      // 正好是这个模型需要的形式, 不用换。
      std::vector<Eigen::Isometry3d> Tv0(na), Tv;
      for (int k = 0; k < na; k++) Tv0[k] = A.frames[ia[k]].T_w_v;
      Eigen::Isometry3d ext_in = A.T_v_l, ext_out = A.T_v_l;
      // 视觉 spec 换成带外参的形式 (T_c_v = T_v_c^-1 是常量)
      std::vector<ialign::VisReprojSpecExt> vspec_e;
      vspec_e.reserve(vcorr.size());
      for (const auto& vc : vcorr) {
        ialign::VisReprojSpecExt vs;
        vs.i = vc.i;
        vs.j = vc.j;
        vs.p_i = vc.p_i;
        vs.T_c_v = cams[vc.cam].T_v_c.inverse().matrix();
        vs.fx = cams[vc.cam].fx;
        vs.fy = cams[vc.cam].fy;
        vs.cx = cams[vc.cam].cx;
        vs.cy = cams[vc.cam].cy;
        vs.u = vc.u;
        vs.v = vc.v;
        vspec_e.push_back(vs);
      }
      cst = ialign::optimizeSubmapCeresExt(Tv, ext_out, Tv0, ext_in, rel,
                                          Eigen::Vector3d::UnitZ(), a_center, co, vspec_e);
      if (cst.ok) {
        Tout.resize(na);
        for (int k = 0; k < na; k++) Tout[k] = Tv[k] * ext_out;
        // ---- 外参优化前后 ----
        const auto rpy = [](const Eigen::Isometry3d& T) {
          const Eigen::Matrix3d& R = T.linear();
          return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI,
                                 -std::asin(std::max(-1.0, std::min(1.0, R(2, 0)))) * 180.0 / M_PI,
                                 std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI);
        };
        const Eigen::Vector3d e0 = rpy(ext_in), e1 = rpy(ext_out);
        const Eigen::Vector3d t0 = ext_in.translation(), t1 = ext_out.translation();
        const Eigen::AngleAxisd da(ext_in.linear().transpose() * ext_out.linear());
        printf("  ---- 雷达外参 T_v_l (yaml 初值 -> BA 优化后) ----\n"
               "    roll  %+8.4f -> %+8.4f  deg   (变动 %+.4f)\n"
               "    pitch %+8.4f -> %+8.4f  deg   (变动 %+.4f)\n"
               "    yaw   %+8.4f -> %+8.4f  deg   (变动 %+.4f)\n"
               "    tx    %+8.4f -> %+8.4f  m     (变动 %+.4f)\n"
               "    ty    %+8.4f -> %+8.4f  m     (变动 %+.4f)\n"
               "    tz    %+8.4f -> %+8.4f  m     (变动 %+.4f)\n"
               "    总旋转变动 = %.4f deg   (外参平移%s优化)\n",
               e0.x(), e1.x(), e1.x() - e0.x(), e0.y(), e1.y(), e1.y() - e0.y(), e0.z(), e1.z(),
               e1.z() - e0.z(), t0.x(), t1.x(), t1.x() - t0.x(), t0.y(), t1.y(), t1.y() - t0.y(),
               t0.z(), t1.z(), t1.z() - t0.z(), std::abs(da.angle()) * 180.0 / M_PI,
               o.opt_ext >= 2 ? "已" : "未");
      }
    } else {
      cst = ialign::optimizeSubmapCeres(Tout, Ta_w0, rel, Eigen::Vector3d::UnitZ(), a_center, co,
                                       vspec);
    }
    if (!cst.ok) {
      printf("  !! Ceres 没求解成功 (帧=%d 帧间约束=%d 视觉=%d), 沿用 INS 位姿\n", cst.n_frames,
             cst.n_rel, cst.n_vis);
    } else {
      std::vector<double> dsp;
      for (int k = 0; k < na; k++) {
        dsp.push_back((Tout[k].translation() - Ta_w0[k].translation()).norm());
      }
      std::sort(dsp.begin(), dsp.end());
      printf("  Ceres: 帧=%d 帧间=%d 视觉=%d 剔除=%d  cost %.4g -> %.4g\n"
             "    每帧位移: 中位=%.3f p90=%.3f max=%.3f m\n",
             cst.n_frames, cst.n_rel, cst.n_vis, cst.n_removed, cst.cost_before, cst.cost_after,
             dsp[dsp.size() / 2], dsp[std::min(dsp.size() - 1, dsp.size() * 9 / 10)], dsp.back());
      Ta_w = Tout;
    }
  }

  // ---- 拼 A 的 submap (BA 前/后各一份, 便于直接对比) ----
  std::vector<Eigen::Vector4d> sm, sm_odd, sm_even, sm_ins;
  for (int k = 0; k < na; k++) {
    if (apc[k].points.empty()) continue;
    auto& half = (k % 2 == 0) ? sm_even : sm_odd;
    for (const auto& p : apc[k].points) {
      sm.push_back(Ta_w[k] * p);
      half.push_back(Ta_w[k] * p);
      if (o.ba) sm_ins.push_back(Ta_w0[k] * p);
    }
  }
  if (sm.empty()) {
    std::cerr << "  !! A 的 submap 是空的\n";
    return 1;
  }
  {
    std::vector<double> none;
    voxelDownsample(sm, none, o.submap_voxel);
    none.clear();
    voxelDownsample(sm_odd, none, o.submap_voxel);
    none.clear();
    voxelDownsample(sm_even, none, o.submap_voxel);
  }
  if (o.ba && !sm_ins.empty()) {
    std::vector<double> none;
    voxelDownsample(sm_ins, none, o.submap_voxel);
    savePts(fs::path(o.out) / "submap_A_ins.pcd", sm_ins);
    printf("  submap_A_ins.pcd: %zu 点 (BA 前, 纯 INS 位姿) —— 和 submap_A.pcd 叠着看 BA 改了什么\n",
           sm_ins.size());
  }
  // ---- A 的位姿 (BA 前/后) ----
  // 用来判断 BA 是**刚性平移**还是**形变**: 前者只是规范系问题(重新选锚定即可),
  // 后者说明各锚点的 submap 彼此真的不自洽, 稀疏锚点方案不成立。
  {
    std::ofstream f((fs::path(o.out) / "poses_A.csv").string());
    f << "k,fid,x0,y0,z0,yaw0,x1,y1,z1,yaw1\n";
    const auto yawOf = [](const Eigen::Isometry3d& T) {
      return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
    };
    for (int k = 0; k < na; k++) {
      const auto& t0 = Ta_w0[k].translation();
      const auto& t1 = Ta_w[k].translation();
      f << k << ',' << ia[k] << ',' << t0.x() << ',' << t0.y() << ',' << t0.z() << ','
        << yawOf(Ta_w0[k]) << ',' << t1.x() << ',' << t1.y() << ',' << t1.z() << ','
        << yawOf(Ta_w[k]) << '\n';
    }
  }
  savePts(fs::path(o.out) / "submap_A.pcd", sm);
  savePts(fs::path(o.out) / "submap_A_odd.pcd", sm_odd);
  savePts(fs::path(o.out) / "submap_A_even.pcd", sm_even);
  printf("  submap_A: %zu 点 (奇 %zu / 偶 %zu)\n", sm.size(), sm_odd.size(), sm_even.size());
  if (o.dyn_filter) {
    printf("  动态剔除: 有标注的帧=%ld 没匹配到=%ld 共删点=%ld\n", dyn_frames, dyn_nomatch, dyn_sum);
    if (dyn_nomatch > dyn_frames) {
      printf("    !! 没匹配到标注的帧比有标注的还多 —— 时间戳对不上或 annotations 缺失,\n"
             "       这种情况下等于没剔, 而 target 里会留着车的残影。\n");
    }
  }

  auto tgt = std::make_shared<gtsam_points::PointCloudCPU>();
  tgt->add_points(sm);
  addCovs(tgt, ce, o.num_threads);
  auto vm = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(o.tgt_voxel);
  vm->insert(*tgt);
  gtsam_points::KdTree tree(tgt->points, tgt->size());

  // 地面高度图: 用 submap 建, 后面所有 nn 都用同一个基准
  Eigen::Vector2d mn(1e18, 1e18), mx(-1e18, -1e18);
  for (const auto& p : sm) {
    mn = mn.cwiseMin(p.head<2>());
    mx = mx.cwiseMax(p.head<2>());
  }
  const auto gm = ialign::buildGroundMap({{tgt.get(), Eigen::Isometry3d::Identity()}}, mn.x(), mn.y(),
                                        mx.x(), mx.y(), 8.0);
  const double kOccCell = 2.0;
  const auto occ = buildOcc(sm, kOccCell);
  printf("  覆盖门: submap 占据 %zu 个 %.0fm 格 —— 只统计落在这些格里的源点\n", occ.size(), kOccCell);

  // ---- target 自身有多糊 (奇偶两半互测) ----
  // **这一项必须先看**: 配准精度的上限就是 target 自身的清晰度。它糊到 0.1,
  // 就别指望把 B 对到 0.03 —— 这个项目里反复验证过。
  double tgt_ghost = -1.0;
  if (sm_odd.size() > 5000 && sm_even.size() > 5000) {
    auto ca = std::make_shared<gtsam_points::PointCloudCPU>();
    auto cb = std::make_shared<gtsam_points::PointCloudCPU>();
    ca->add_points(sm_even);
    cb->add_points(sm_odd);
    gtsam_points::KdTree ta(ca->points, ca->size());
    tgt_ghost = aboveGroundNN(*ca, gm, ta, *cb, Eigen::Isometry3d::Identity(), o.z_above, 5000, 3.0,
                              &occ, kOccCell);
  }
  printf("  **target 自身重影 (奇偶互测, 地面以上) = %.3f m**\n"
         "    配准精度的上限就是这个数。下面每帧的 nn 若明显小于它, 那是往模糊里塞出来的假象。\n",
         tgt_ghost);

  // ---- B 的帧逐一配准 ----
  struct Res {
    long fid = -1;
    double inl = -1, corr = -1, nn0 = -1, nn1 = -1;
    long n_used = 0, n_out = 0;   // 参与 nn 的源点数 / 被覆盖门剔掉的
    bool ok = false;
    Eigen::Isometry3d T0 = Eigen::Isometry3d::Identity();  // INS
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();  // 配准后
  };
  std::vector<Res> res(ib.size());
  std::atomic<int> ndump{0};

#pragma omp parallel for num_threads(o.num_threads) schedule(dynamic)
  for (std::int64_t k = 0; k < static_cast<std::int64_t>(ib.size()); k++) {
    auto& r = res[k];
    r.fid = ib[k];
    r.T0 = B.frames[ib[k]].T_w_l;
    r.T1 = r.T0;

    ialign::PcdCloud pc;
    if (!loadFrame(B.frames[ib[k]].pcd_path, o, pc)) continue;
    auto src = std::make_shared<gtsam_points::PointCloudCPU>();
    src->add_points(pc.points);
    addCovs(src, ce, 1);

    // VGICP (体素高斯 target; 精度上限就是体素尺度, 所以后面还要 GICP 精化)
    gtsam::Values vals;
    vals.insert(0, gtsam::Pose3(r.T0.matrix()));
    gtsam::NonlinearFactorGraph g;
    auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, vm, src);
    g.add(f);
    Eigen::Isometry3d T = r.T0;
    try {
      gtsam_points::LevenbergMarquardtExtParams lm;
      lm.setMaxIterations(o.iters);
      vals = gtsam_points::LevenbergMarquardtOptimizerExt(g, vals, lm).optimize();
      T = Eigen::Isometry3d(vals.at<gtsam::Pose3>(0).matrix());
    } catch (const std::exception&) {
      continue;
    }
    r.inl = f->inlier_fraction();

    // GICP 精化: KdTree 找真实最近邻, 没有体素离散化上限
    if (o.fine && r.inl >= o.min_inlier) {
      gtsam::Values v2;
      v2.insert(0, gtsam::Pose3(T.matrix()));
      gtsam::NonlinearFactorGraph g2;
      auto ff = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(
        gtsam::Pose3(), 0, tgt, src,
        std::shared_ptr<gtsam_points::NearestNeighborSearch>(
          &tree, [](gtsam_points::NearestNeighborSearch*) {}));
      ff->set_max_correspondence_distance(o.fine_corr);
      g2.add(ff);
      try {
        gtsam_points::LevenbergMarquardtExtParams lm2;
        lm2.setMaxIterations(o.fine_iters);
        v2 = gtsam_points::LevenbergMarquardtOptimizerExt(g2, v2, lm2).optimize();
        T = Eigen::Isometry3d(v2.at<gtsam::Pose3>(0).matrix());
      } catch (const std::exception&) {
      }
    }

    const Eigen::Isometry3d Ta = o.clamp_planar ? clampPlanar(T, r.T0) : T;
    r.corr = (Ta.translation().head<2>() - r.T0.translation().head<2>()).norm();
    r.nn0 = aboveGroundNN(*tgt, gm, tree, *src, r.T0, o.z_above, 3000, 3.0, &occ, kOccCell,
                          &r.n_used, &r.n_out);
    r.nn1 = aboveGroundNN(*tgt, gm, tree, *src, Ta, o.z_above, 3000, 3.0, &occ, kOccCell);
    r.ok = r.inl >= o.min_inlier && r.corr <= o.max_corr &&
           !(r.nn0 > 0 && r.nn1 > 0 && r.nn1 > r.nn0);
    if (r.ok) r.T1 = Ta;

    // ---- 落盘: 分数在最前, 所以 ls 天然按质量排序 ----
    if (ndump.load() < o.dump_max) {
      const int seq = ndump++;
      if (seq < o.dump_max) {
        char base[192];
        std::snprintf(base, sizeof(base), "nn%03d_g%+04d_%s_f%05ld_inl%02d_corr%03d",
                      static_cast<int>(std::lround(std::max(0.0, r.nn1) * 100)),
                      static_cast<int>(std::lround((r.nn0 - r.nn1) * 100)), r.ok ? "OK" : "REJ",
                      r.fid, static_cast<int>(std::lround(std::max(0.0, r.inl) * 100)),
                      static_cast<int>(std::lround(std::min(9.99, std::max(0.0, r.corr)) * 100)));
        const fs::path fd = fs::path(o.out) / "frames";
        std::vector<Eigen::Vector4d> vb, va;
        vb.reserve(src->size());
        va.reserve(src->size());
        for (std::size_t i = 0; i < src->size(); i++) {
          vb.push_back(r.T0 * src->points[i]);
          va.push_back(Ta * src->points[i]);
        }
        // savePts(fd / (std::string(base) + "_before.pcd"), vb);
        savePts(fd / (std::string(base) + "_after.pcd"), va);
        char la[96], lb[80], lm3[240];
        std::snprintf(la, sizeof(la), "A SUBMAP (%zu PTS, SELF-GHOST %.3FM)", tgt->size(),
                      std::max(0.0, tgt_ghost));
        std::snprintf(lb, sizeof(lb), "B FRAME %ld", r.fid);
        std::snprintf(lm3, sizeof(lm3),
                      "%s INLIER=%.2f CORR=%.2FM | ABOVE-GROUND NN %.3F->%.3F M (TGT SELF %.3F)",
                      r.ok ? "ACCEPTED" : "REJECTED", std::max(0.0, r.inl), r.corr, r.nn0, r.nn1,
                      std::max(0.0, tgt_ghost));
        ialign::LoopImageParams ip;
        ip.res = 0.08;
        ip.tol = 0.15;
        ip.z_above = o.z_above;
        ialign::renderTopDownPair(*tgt, *src, r.T0, Ta, ip, la, lb, lm3,
                                  (fd / (std::string(base) + ".png")).string());
      }
    }
  }

  // ---- 汇总 ----
  std::vector<double> vi, vc, v0, v1, vg;
  int nok = 0;
  for (const auto& r : res) {
    if (r.inl < 0) continue;
    vi.push_back(r.inl);
    vc.push_back(r.corr);
    if (r.nn0 > 0) v0.push_back(r.nn0);
    if (r.nn1 > 0) v1.push_back(r.nn1);
    if (r.nn0 > 0 && r.nn1 > 0) vg.push_back(r.nn0 - r.nn1);
    if (r.ok) nok++;
  }
  const auto med = [](std::vector<double>& v) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  };
  const auto p90 = [](std::vector<double>& v) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, v.size() * 9 / 10)];
  };
  {
    long su = 0, so = 0;
    for (const auto& r : res) { su += r.n_used; so += r.n_out; }
    printf("\n  覆盖门: 参与 nn 的源点=%ld  因超出 A 的覆盖被剔=%ld (%.0f%%)\n"
           "    这个比例高说明 B 的帧看到很多 A 没覆盖的地方 —— 那是正常的(单帧 60m 量程 vs\n"
           "    局部 submap), 但不加这道门就会把覆盖差异算成错位(第一版因此把 0.7 报成 1.4)。\n",
           su, so, (su + so) ? 100.0 * so / (su + so) : 0.0);
  }
  printf("\n=== 结果 ===\n"
         "  配上=%d/%zu  inlier 中位=%.3f  修正量 中位=%.3f p90=%.3f m\n"
         "  地面以上 nn: 配准前 中位=%.3f -> 配准后 中位=%.3f   (改善中位=%.3f m)\n"
         "  target 自身重影=%.3f m  <- 上面配准后的 nn 若小于它, 是往模糊里塞\n",
         nok, ib.size(), med(vi), med(vc), p90(vc), med(v0), med(v1), med(vg), tgt_ghost);

  // 轨迹畸变: 逐帧独立配准会不会把 B 的轨迹搞成锯齿。
  // "地图对地图的最近邻"这个指标对轨迹畸变是全盲的, 必须单独量。
  {
    std::vector<double> warp;
    for (std::size_t k = 1; k < res.size(); k++) {
      if (!res[k].ok || !res[k - 1].ok) continue;
      if (res[k].fid != res[k - 1].fid + 1) continue;   // 只看真正相邻的帧
      const Eigen::Isometry3d ins = res[k - 1].T0.inverse() * res[k].T0;
      const Eigen::Isometry3d opt = res[k - 1].T1.inverse() * res[k].T1;
      warp.push_back((opt.translation() - ins.translation()).norm());
    }
    if (!warp.empty()) {
      printf("  轨迹畸变(相邻帧相对位姿偏离 INS): 中位=%.3f p90=%.3f m  (%zu 对)\n"
             "    逐帧独立配准没有任何平滑约束, 所以这一项要单独看 —— 它和上面的 nn 是\n"
             "    两个方向的代价: nn 变好但轨迹被扭, 等于拿轨迹质量换对齐。\n",
             med(warp), p90(warp), warp.size());
    }
  }

  // ---- 导出 B 的配准前/后 + csv ----
  {
    std::vector<Eigen::Vector4d> pb, pa;
    for (const auto& r : res) {
      if (r.inl < 0) continue;
      ialign::PcdCloud pc;
      if (!loadFrame(B.frames[r.fid].pcd_path, o, pc)) continue;
      for (const auto& p : pc.points) {
        pb.push_back(r.T0 * p);
        pa.push_back((r.ok ? r.T1 : r.T0) * p);
      }
    }
    // savePts(fs::path(o.out) / "B_before.pcd", pb);
    savePts(fs::path(o.out) / "B_after.pcd", pa);
    printf("\n  B_before.pcd / B_after.pcd: %zu 点\n", pb.size());
  }
  {
    std::ofstream f((fs::path(o.out) / "result.csv").string());
    f << "fid,ok,inlier,corr_m,nn_before,nn_after,gain,x0,y0,z0,x1,y1,z1\n";
    for (const auto& r : res) {
      if (r.inl < 0) continue;
      f << r.fid << ',' << (r.ok ? 1 : 0) << ',' << r.inl << ',' << r.corr << ',' << r.nn0 << ','
        << r.nn1 << ',' << (r.nn0 > 0 && r.nn1 > 0 ? r.nn0 - r.nn1 : 0.0) << ','
        << r.T0.translation().x() << ',' << r.T0.translation().y() << ',' << r.T0.translation().z()
        << ',' << r.T1.translation().x() << ',' << r.T1.translation().y() << ','
        << r.T1.translation().z() << '\n';
    }
  }
  const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  printf("\n  怎么看:\n"
         "    1) 先叠 submap_A_odd.pcd 和 submap_A_even.pcd —— target 自己有多糊(%.3fm),\n"
         "       它是后面一切的上限;\n"
         "    2) frames/ 下按名字排序就是按配准后 nn 排序, ls -r 从最差看起;\n"
         "       文件名 nn<配准后x100>_g<改善量x100>_判定_f<帧号>_inl_corr;\n"
         "       png 上下两行=配准前/后, 左右两列=全部点/地面以上, **看地面以上那一行**;\n"
         "    3) 叠 B_before.pcd 和 B_after.pcd 看整体改变;\n"
         "    4) result.csv 每帧一行, 拿它找哪些帧配坏了。\n"
         "  输出: %s   %.0fs\n",
         tgt_ghost, o.out.c_str(), sec);
  std::cout << " optimization end ...... "  << std::endl;
  return 0;
}

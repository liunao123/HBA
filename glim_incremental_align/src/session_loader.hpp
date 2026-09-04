#pragma once
// -----------------------------------------------------------------------------
// WG_wuling incremental_mapping 数据集加载器
// 逻辑对齐 lt-mapper/session_align/include/session_align/clip_loader.hpp
//
// 目录结构 (一个 session = 一个 <session_root>):
//   <session_root>/clips/<clip>/egomotions/<ts>.json                 位姿 (默认用这个)
//   <session_root>/clips/<clip>/localized/<ts>.json                  位姿 (INS/GNSS)
//   <session_root>/clips/<clip>/sensors/fuse_lidar/<ts>.pcd          点云 (lidar 系)
//   <session_root>/clips/<clip>/calibration/vehicle_params.json      utm_center
//   <session_root>/clips/<clip>/calibration/fuse_lidar_2_vehicle_extrinsics.yaml
//
// 关键约定:
//   - position 是 **相对 utm_center 的** UTM 坐标 (米); 三个 session 共享同一 utm_center,
//     所以它们的位姿天然在同一世界系里, 不需要求 session 间的未知变换。
//     (egomotions 的 position 同样是相对 UTM, 不是里程计局部系 —— 它与同帧
//      navi_traj_interp_utm[0] 完全一致。)
//   - 点云在 lidar 系, p_w = T_w_v * T_v_l * p_lidar
//
// 为什么默认 egomotions + orientation (本机实测, jjst2 前 6 clip / 348 帧):
//   |localized.pos - egomotions.pos|      中位 0.0087m  最大 0.137m   -> 位置基本等价
//   localized.euler vs egomotions.quat    中位 0.0012°  最大 0.056°   -> 姿态等价
//   localized.quat  vs egomotions.quat    中位 4.44°    最大 25.5°    -> localized 的四元数是坏的
//   即 localized.orientation 不可用; egomotions.orientation 自洽, 直接读它最干净。
// -----------------------------------------------------------------------------

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ialign {

namespace fs = std::filesystem;
using json = nlohmann::json;

struct Frame {
  double ts = 0.0;
  Eigen::Isometry3d T_w_v = Eigen::Isometry3d::Identity();  // 世界 <- vehicle
  Eigen::Isometry3d T_w_l = Eigen::Isometry3d::Identity();  // 世界 <- lidar
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  std::string pcd_path;
  int clip_id = -1;  // 只有同一 clip 内时间才连续
};

struct SessionData {
  int index = 0;
  std::string name;
  fs::path root;
  Eigen::Vector2d utm_center{0, 0};
  /// 这个 session 的位姿为了并到**基准 session 的世界系**而被平移的量 (= 原 utm_center
  /// - base_utm)。统一世界系时 utm_center 会被改写成 base_utm, 原值就丢了 —— 而把优化后
  /// 的位姿写回原始 egomotions 目录时必须减掉它, 否则写出去的 position 和该 clip 自己的
  /// calibration/vehicle_params.json 里的 utm_center 不在同一个系里, 差几百米还不报错。
  Eigen::Vector2d utm_shift{0, 0};
  Eigen::Isometry3d T_v_l = Eigen::Isometry3d::Identity();
  std::vector<Frame> frames;  // 按 clip、再按时间排序
};

struct LoadOptions {
  std::string attitude_from = "quat";   // "quat" | "euler"
  std::string pose_dir = "egomotions";  // "egomotions" | "localized"
  // 数据组织方式:
  //   "clips"    : WG_wuling 那套 —— <session>/clips/<clip>/{egomotions,sensors/fuse_lidar,calibration}
  //   "keyframe" : dwm_data 那套 —— <session>/{pointclouds, sparse/vehicle_lidar_pose}
  std::string data_mode = "clips";
  // keyframe 模式下位姿取自哪里:
  //   "sparse" : sparse/vehicle_lidar_pose —— **已经是优化输出**, 实测跨 session 一致性 0.044m
  //   "odoms"  : odoms/ —— 更粗的一版, 与 sparse 差 0.9m (见 loadOdomsPoses 里的实测)
  // 为什么需要 odoms: sparse 已经准到 0.044m, 远低于单次配准的噪声地板(~0.20m),
  // 信噪比 < 1, 这套增量对齐流程在它上面只能持平、不可能提升 —— 拿它验证不了任何东西。
  std::string pose_src = "sparse";
  // clips 模式用 sensors/ 下的哪个雷达目录。本来硬编码在代码里, 提成选项 ——
  // 实测 fuse_lidar 每帧 200023 点、front_lidar 92341 点(46%), 而两者的**外参逐比特相同**
  // (即 fuse 的点本来就在 front_lidar 系里), 所以切换时不用动外参。
  std::string lidar_dir = "fuse_lidar";
};

// -----------------------------------------------------------------------------
// keyframe 模式 (dwm_data)
//
// 目录结构 (一个 session = 一个 <session_root>, 没有 clip 层):
//   <session_root>/pointclouds/<stem>.pcd                      点云 (binary_compressed!)
//   <session_root>/sparse/vehicle_lidar_pose/<stem>.yaml        位姿
//
// <stem> 形如 "1000_1776466462.900" = "<序号>_<时间戳>", 点云和位姿一一同名。
//
// 位姿 yaml:
//   timestamp:  1776466462.9
//   pose_utm:   16 个数, **行主序** 4x4 齐次矩阵
//   offset_utm: 3 个数, 世界系原点在 UTM 里的位置
//
// 与 clips 模式的三处关键差异:
//   1. pose_utm 是 **T_world_lidar**, 不是车体位姿 —— 所以 T_v_l = I, 不读外参。
//      (雷达外参 calib/extrinsics/0_hesai128_2_vehicle_Extrinsics.yaml 里有 2.12m 的 z 偏移
//       和 90 度 yaw, 若误当成车体位姿去乘外参, 会引入 2m 高差 + 90 度旋转。)
//   2. 点云是 binary_compressed (LZF + 按字段连续排布), 需要 pcd_io 里那段解压。
//   3. 没有 clip 概念, 整个 session 是一条连续轨迹 —— clip_id 全部记 0。
//      这一点影响 groupKeyframes: 它靠 clip_id 变化和相邻间距切"空间连续段",
//      keyframe 模式下只剩间距判据(kf_gap_max), 数据本身若有跳变要靠它拦。
//
// 排序必须按**序号**而不是字符串: "100_..." 的字符串序在 "2_..." 之前, 直接排会把
// 轨迹顺序打乱, 而后面所有"相邻关键帧"的逻辑都依赖顺序正确。
// -----------------------------------------------------------------------------
inline bool isKeyframeSession(const fs::path& p) {
  return fs::is_directory(p / "pointclouds") && fs::is_directory(p / "sparse" / "vehicle_lidar_pose");
}

/// @brief 从 "1000_1776466462.900" 里取出前面的序号; 取不到返回 -1
inline long keyframeSeq(const std::string& stem) {
  const auto us = stem.find('_');
  if (us == std::string::npos) return -1;
  try {
    return std::stol(stem.substr(0, us));
  } catch (...) {
    return -1;
  }
}

/// @brief 读一个 vehicle_lidar_pose yaml。手写解析而不用 cv::FileStorage ——
///        这是纯 YAML 序列, 不是 OpenCV 的 %YAML:1.0 格式, FileStorage 读不了。
inline bool readKeyframePose(const fs::path& file, Eigen::Isometry3d& T_w_l, double& ts, Eigen::Vector2d& off) {
  std::ifstream ifs(file.string());
  if (!ifs) return false;
  std::vector<double> pose, offset;
  std::string line, section;
  ts = 0.0;
  while (std::getline(ifs, line)) {
    // 去掉行尾 \r (数据可能来自 Windows)
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos) continue;
    const std::string trimmed = line.substr(first);

    if (trimmed.rfind("- ", 0) == 0) {   // 列表项
      try {
        const double v = std::stod(trimmed.substr(2));
        if (section == "pose_utm") pose.push_back(v);
        else if (section == "offset_utm") offset.push_back(v);
      } catch (...) {
      }
      continue;
    }
    // key: 或 key: value
    const auto colon = trimmed.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trimmed.substr(0, colon);
    const std::string val = trimmed.substr(colon + 1);
    const auto vf = val.find_first_not_of(" \t");
    if (key == "timestamp") {
      section.clear();
      if (vf != std::string::npos) {
        try {
          ts = std::stod(val.substr(vf));
        } catch (...) {
        }
      }
    } else {
      section = key;  // pose_utm / offset_utm, 值在后续的 "- " 行里
    }
  }
  if (pose.size() != 16) {
    std::cerr << "[load] " << file << " 的 pose_utm 有 " << pose.size() << " 个数, 应为 16\n";
    return false;
  }
  // 行主序 -> Eigen 默认列主序, 逐元素填, 不能直接 Map
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) M(r, c) = pose[r * 4 + c];
  }
  T_w_l = Eigen::Isometry3d(M);
  off = offset.size() >= 2 ? Eigen::Vector2d(offset[0], offset[1]) : Eigen::Vector2d(0, 0);
  return true;
}

// -----------------------------------------------------------------------------
// odoms/ 位姿 (--pose_src odoms)
//
//   <session_root>/odoms/<同名 stem>.yaml
//     position:     3 个数, **局部 ENU**, 首帧为 (0,0,0)
//     pose:         9 个数, 3x3 旋转, **列主序** —— 注意和 pose_utm 的行主序相反!
//                   (实测 5 帧: 按列主序解出的 yaw 与同文件 heading 精确到 5 位小数一致;
//                    按行主序解出的是它的相反数。同一份数据里两种约定, 搞错就是整个
//                    session 的航向镜像, 而点云会"看起来只是配不准", 不会报错。)
//     heading:      ENU yaw (逆时针, 从东起算), 度; 与上面的旋转矩阵一致
//     position_lla, enu_velocity, speed, timestamp
//
// odoms 是**车体**在局部 ENU 下的位姿, 而 pose_utm 是**雷达**在 UTM 下的位姿。要把
// odoms 用起来, 需要两个未知量: ENU->UTM 的刚体变换, 以及车体->雷达的外参 T_v_l。
// 两者都从两份文件的**整段**配对里一次性拟合出来 (每个 session 各 6 自由度), 而不是
// 逐帧去改 —— 所以逐帧那 0.9m 的差异被完整保留下来, 那正是要测的"输入误差"。
//
// 实测 260418_2 (3697 帧 / 8283m):
//   ENU->UTM 旋转 = -1.266deg, 恰等于该处的 UTM 网格收敛角
//   -(120.62-123)*sin(31.43) = -1.24deg  -> 拟合到的是真实坐标系差异, 不是在吸收误差
//   逐帧残差 中位 0.908m, **在 8.3km 上不增长**(分段 0.83~1.11), 也不是车体系常向量
//   (转回车体系后均值仅 0.178m 而标准差 0.57/0.72m, 扣掉常向量后仍是 0.895m)
//   => 不是漂移、不是杆臂, 是 sparse 那一版修掉的一段有界误差。
// -----------------------------------------------------------------------------
inline bool readOdomPose(const fs::path& file, Eigen::Isometry3d& T_enu_v) {
  std::ifstream ifs(file.string());
  if (!ifs) return false;
  std::vector<double> pos, rot;
  std::string line, section;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos) continue;
    const std::string trimmed = line.substr(first);
    if (trimmed.rfind("- ", 0) == 0) {
      try {
        const double v = std::stod(trimmed.substr(2));
        if (section == "position") pos.push_back(v);
        else if (section == "pose") rot.push_back(v);
      } catch (...) {
      }
      continue;
    }
    const auto colon = trimmed.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trimmed.substr(0, colon);
    // 只有 position / pose 需要收集列表; 其余的键(含带值的 timestamp/heading)一律清掉 section,
    // 否则 position_lla 的三个数会被接到 position 后面
    section = (key == "position" || key == "pose") ? key : std::string();
  }
  if (pos.size() != 3 || rot.size() != 9) return false;
  Eigen::Matrix3d M;
  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) M(r, c) = rot[c * 3 + r];   // 列主序
  }
  // ---- NED -> ENU: 左乘 S ----
  // position 是 ENU (拟合出的 ENU->UTM 旋转 -1.235度 恰是网格收敛角, 证实 x=东),
  // 但**姿态用的是另一套约定**: heading 是北起顺时针。同一个文件里两种约定。
  // 实测判据 (用"姿态与实际位移方向之差"的圆标准差, vlp 的基准是 0.50 度):
  //     yaw=heading      104.58 度   <- 直接用, 完全错
  //     yaw=90-heading     0.48 度   <- 正确
  // 而按列主序读出的 M 满足 atan2(M(1,0),M(0,0)) = heading, 所以 M 是 NED 系的。
  // S = [[0,1,0],[1,0,0],[0,0,-1]] (det=+1) 把世界侧从 NED 转到 ENU;
  // 车体侧还差一个常量翻转(S*M 的 roll 是 180 度), 那个由拟合出的 T_v_l 吸收 —— 只要
  // 它逐帧一致就没问题, loadOdomsPoses 里会检查这个一致性。
  Eigen::Matrix3d S;
  S << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Eigen::Matrix3d R = S * M;
  // 数值上不一定严格正交, 投影回 SO(3)
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d Rp = svd.matrixU() * svd.matrixV().transpose();
  if (Rp.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    Rp = svd.matrixU() * V.transpose();
  }
  T_enu_v.setIdentity();
  T_enu_v.linear() = Rp;
  T_enu_v.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
  return true;
}

/// @brief 最近的旋转矩阵 (对一组 R 求平均后投影回 SO(3))
inline Eigen::Matrix3d projectSO3(const Eigen::Matrix3d& M) {
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    R = svd.matrixU() * V.transpose();
  }
  return R;
}

/// @brief 用 odoms 的位姿替换 out.frames 里的 T_w_v/T_w_l。out 必须已经由
///        loadKeyframeSession 填好 (需要它的 T_w_l 来拟合 ENU->UTM 和 T_v_l)。
inline bool loadOdomsPoses(const fs::path& session_root, SessionData& out) {
  const fs::path od = session_root / "odoms";
  if (!fs::is_directory(od)) {
    std::cerr << "[load] --pose_src odoms 需要 " << od << "\n";
    return false;
  }
  // 逐帧读 odoms; stem 与点云/pose_utm 同名
  std::vector<Eigen::Isometry3d> Te(out.frames.size());
  std::vector<char> ok(out.frames.size(), 0);
  std::size_t n_ok = 0;
  for (std::size_t i = 0; i < out.frames.size(); i++) {
    const std::string stem = fs::path(out.frames[i].pcd_path).stem().string();
    if (readOdomPose(od / (stem + ".yaml"), Te[i])) {
      ok[i] = 1;
      n_ok++;
    }
  }
  if (n_ok < 20) {
    std::cerr << "[load] odoms 只读出 " << n_ok << " 帧, 放弃\n";
    return false;
  }

  // ---- 1) 拟合 ENU -> UTM(本 session 的局部系) 的刚体变换 (Umeyama, 无缩放) ----
  Eigen::Vector3d ce = Eigen::Vector3d::Zero(), cu = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < out.frames.size(); i++) {
    if (!ok[i]) continue;
    ce += Te[i].translation();
    cu += out.frames[i].T_w_l.translation();
  }
  ce /= static_cast<double>(n_ok);
  cu /= static_cast<double>(n_ok);
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < out.frames.size(); i++) {
    if (!ok[i]) continue;
    H += (Te[i].translation() - ce) * (out.frames[i].T_w_l.translation() - cu).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d Rue = svd.matrixV() * svd.matrixU().transpose();
  if (Rue.determinant() < 0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1;
    Rue = svd.matrixV() * U.transpose();
  }
  Eigen::Isometry3d T_u_e = Eigen::Isometry3d::Identity();
  T_u_e.linear() = Rue;
  T_u_e.translation() = cu - Rue * ce;

  // ---- 2) 拟合车体->雷达外参 T_v_l = mean_i (T_u_e * T_enu_v_i)^-1 * T_w_l_i ----
  Eigen::Matrix3d Rsum = Eigen::Matrix3d::Zero();
  Eigen::Vector3d tsum = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < out.frames.size(); i++) {
    if (!ok[i]) continue;
    const Eigen::Isometry3d E = (T_u_e * Te[i]).inverse() * out.frames[i].T_w_l;
    Rsum += E.linear();
    tsum += E.translation();
  }
  Eigen::Isometry3d T_v_l = Eigen::Isometry3d::Identity();
  T_v_l.linear() = projectSO3(Rsum / static_cast<double>(n_ok));
  T_v_l.translation() = tsum / static_cast<double>(n_ok);

  // ---- 外参必须逐帧一致, 否则"平均"是没有意义的 ----
  // 外参是安装关系, 定义上是常量。如果逐帧估计发散, 说明两边的姿态约定还没对上,
  // 此时元素平均再投影 SO(3) 会给出一个看起来正常的矩阵(曾实测 yaw=136.83 度, 而逐帧
  // 直接算是 179.7 度), 程序照跑不误, 但每帧点云都绕自身原点转错几十度 —— 整张图是
  // 垃圾, 而日志里没有任何异常。所以这里必须硬拦。
  // 只看**旋转**的离散度: 平移那一项的离散度就是 odoms 与 sparse 的位置差异本身
  // (0.9m 量级), 那正是要测的输入误差, 拿它当一致性判据等于在拒绝要测的东西。
  // 门槛按量级定, 而不是"越小越好": 约定错误产生 90~180 度的离散, 而 sparse 那一版
  // 若同时修了姿态, 几度的真实差异是合理的 —— 所以卡在 15 度, 两类分得开。
  {
    std::vector<double> dang;
    dang.reserve(n_ok);
    for (std::size_t i = 0; i < out.frames.size(); i++) {
      if (!ok[i]) continue;
      const Eigen::Isometry3d E = (T_u_e * Te[i]).inverse() * out.frames[i].T_w_l;
      const Eigen::AngleAxisd da(E.linear() * T_v_l.linear().transpose());
      dang.push_back(std::abs(da.angle()) * 180.0 / M_PI);
    }
    std::sort(dang.begin(), dang.end());
    const auto q = [&](double f) { return dang[std::min(dang.size() - 1, static_cast<std::size_t>(f * dang.size()))]; };
    printf("  [odoms] %s: 外参旋转逐帧一致性 中位=%.2f p90=%.2f max=%.2f 度\n",
           out.name.c_str(), q(0.5), q(0.9), dang.back());
    if (q(0.9) > 15.0) {
      std::cerr << "[load] !! 外参旋转 p90 达 " << q(0.9)
                << " 度, 说明 odoms 与 pose_utm 的姿态约定没对上。\n"
                   "        用这个'平均外参'会让每帧点云绕自身原点转错, 整张图是垃圾。中止。\n";
      return false;
    }
  }

  // ---- 3) 替换位姿, 同时量一下与 sparse 版的逐帧差异 (= 本次要测的输入误差) ----
  std::vector<double> dev;
  dev.reserve(n_ok);
  std::size_t n_drop = 0;
  std::vector<Frame> keep;
  keep.reserve(n_ok);
  for (std::size_t i = 0; i < out.frames.size(); i++) {
    if (!ok[i]) {
      n_drop++;
      continue;
    }
    Frame f = out.frames[i];
    const Eigen::Isometry3d T_w_v = T_u_e * Te[i];
    dev.push_back((T_w_v * T_v_l).translation().x() - f.T_w_l.translation().x());
    const double dy = (T_w_v * T_v_l).translation().y() - f.T_w_l.translation().y();
    dev.back() = std::hypot(dev.back(), dy);
    f.T_w_v = T_w_v;
    f.T_w_l = T_w_v * T_v_l;
    const Eigen::Matrix3d& R = f.T_w_l.linear();
    f.roll = std::atan2(R(2, 1), R(2, 2));
    f.pitch = -std::asin(std::max(-1.0, std::min(1.0, R(2, 0))));
    f.yaw = std::atan2(R(1, 0), R(0, 0));
    keep.push_back(std::move(f));
  }
  out.frames.swap(keep);

  std::sort(dev.begin(), dev.end());
  const auto pct = [&](double q) {
    return dev.empty() ? 0.0 : dev[std::min(dev.size() - 1, static_cast<std::size_t>(q * dev.size()))];
  };
  const double yaw_ue = std::atan2(Rue(1, 0), Rue(0, 0)) * 180.0 / M_PI;
  printf("  [odoms] %s: 用 odoms 位姿替换 (%zu 帧, odoms 缺失 %zu 帧已丢弃)\n"
         "          ENU->UTM 旋转=%.3f度 (该纬度的网格收敛角约 -1.24度, 对得上说明拟合到的是\n"
         "          真实坐标系差异而不是在吸收误差)\n"
         "          外参 T_v_l: 平移=(%.3f, %.3f, %.3f)m  yaw=%.2f度\n"
         "          与 sparse 版的逐帧水平偏差: 中位=%.3f p50/p90/max=%.3f/%.3f/%.3f m\n"
         "          ^ 这就是本次要修的输入误差。实测两个 session 是 0.201 / 0.129m 中位,\n"
         "            比 sparse 版的跨 session 误差(0.044m)大 3~5 倍, 所以比 sparse **有得修**;\n"
         "            但它只是**刚好落在**单次配准的噪声地板(~0.20m)上, 信噪比约等于 1 ——\n"
         "            这个测试是勉强可用, 不是决定性的。要决定性结论需要误差更大的数据。\n",
         out.name.c_str(), out.frames.size(), n_drop, yaw_ue,
         T_v_l.translation().x(), T_v_l.translation().y(), T_v_l.translation().z(),
         std::atan2(T_v_l.linear()(1, 0), T_v_l.linear()(0, 0)) * 180.0 / M_PI,
         pct(0.5), pct(0.5), pct(0.9), dev.empty() ? 0.0 : dev.back());
  return !out.frames.empty();
}

inline bool loadKeyframeSession(
  const fs::path& session_root, int index, SessionData& out, const std::string& pose_src = "sparse") {
  out.index = index;
  out.name = session_root.filename().string();
  out.root = session_root;
  out.frames.clear();
  out.T_v_l.setIdentity();   // pose_utm 已经是雷达位姿

  const fs::path cloud_dir = session_root / "pointclouds";
  const fs::path pose_dir = session_root / "sparse" / "vehicle_lidar_pose";
  if (!fs::is_directory(cloud_dir) || !fs::is_directory(pose_dir)) {
    std::cerr << "[load] keyframe 模式需要 pointclouds/ 和 sparse/vehicle_lidar_pose/: " << session_root << "\n";
    return false;
  }

  // 按序号排序 —— 字符串序会把 "100_" 排到 "2_" 前面, 打乱轨迹顺序
  std::vector<std::pair<long, fs::path>> pf;
  for (const auto& e : fs::directory_iterator(pose_dir)) {
    if (e.path().extension() != ".yaml") continue;
    pf.emplace_back(keyframeSeq(e.path().stem().string()), e.path());
  }
  std::sort(pf.begin(), pf.end(), [](const auto& a, const auto& b) {
    if (a.first != b.first) return a.first < b.first;
    return a.second < b.second;
  });
  if (pf.empty()) {
    std::cerr << "[load] " << pose_dir << " 下没有 yaml\n";
    return false;
  }

  bool got_off = false;
  Eigen::Vector2d off0{0, 0};
  std::size_t n_no_pcd = 0, n_bad_pose = 0;
  for (const auto& [seq, p] : pf) {
    const std::string stem = p.stem().string();
    const fs::path pcd = cloud_dir / (stem + ".pcd");
    if (!fs::exists(pcd)) {
      n_no_pcd++;
      continue;
    }
    Frame f;
    Eigen::Vector2d off;
    if (!readKeyframePose(p, f.T_w_l, f.ts, off)) {
      n_bad_pose++;
      continue;
    }
    if (!got_off) {
      off0 = off;
      got_off = true;
    } else if ((off - off0).norm() > 1e-6) {
      // 同一 session 内 offset 变了就说明位姿不在同一世界系, 后面按距离找邻居会全错
      std::cerr << "[load] offset_utm 在 session 内不一致: " << p << "\n";
      return false;
    }
    if (f.ts <= 0) f.ts = static_cast<double>(seq);
    f.T_w_v = f.T_w_l;   // 没有独立的车体位姿, 两者视为同一个
    // roll/pitch/yaw 只用于诊断打印, 从旋转矩阵反解 (ZYX)
    const Eigen::Matrix3d& R = f.T_w_l.linear();
    f.roll = std::atan2(R(2, 1), R(2, 2));
    f.pitch = -std::asin(std::max(-1.0, std::min(1.0, R(2, 0))));
    f.yaw = std::atan2(R(1, 0), R(0, 0));
    f.pcd_path = pcd.string();
    f.clip_id = 0;   // keyframe 模式没有 clip 层, 整个 session 一条连续轨迹
    out.frames.push_back(std::move(f));
  }
  if (n_no_pcd || n_bad_pose) {
    printf("  [keyframe] %s: 跳过 缺点云=%zu 位姿解析失败=%zu\n", out.name.c_str(), n_no_pcd, n_bad_pose);
  }
  out.utm_center = off0;
  if (out.frames.empty()) return false;
  // odoms 版必须在这之后做 —— 它要用上面读到的 T_w_l 去拟合 ENU->UTM 和外参
  if (pose_src == "odoms") return loadOdomsPoses(session_root, out);
  if (pose_src != "sparse") {
    std::cerr << "[load] --pose_src 只认 sparse|odoms, 收到: " << pose_src << "\n";
    return false;
  }
  return true;
}

// ZYX (Rz*Ry*Rx)
inline Eigen::Matrix3d rotFromRPY(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}

/// @brief 读 OpenCV (%YAML:1.0) 外参
inline bool readExtrinsics(const fs::path& file, Eigen::Isometry3d& T) {
  cv::FileStorage cfs(file.string(), cv::FileStorage::READ);
  if (!cfs.isOpened()) return false;
  cv::Mat q, t;
  cfs["r_quaternion_wxyz"] >> q;
  cfs["t_metric_xyz"] >> t;
  cfs.release();
  if (q.empty() || t.empty()) return false;

  Eigen::Quaterniond qe(q.at<double>(0), q.at<double>(1), q.at<double>(2), q.at<double>(3));
  T.setIdentity();
  T.linear() = qe.normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));
  return true;
}

inline bool isClip(const fs::path& p, const std::string& pose_dir) {
  return fs::is_directory(p / pose_dir) && fs::is_directory(p / "sensors" / "fuse_lidar") &&
         fs::is_directory(p / "calibration");
}

/// @brief 目录名里带 '.' = 数据还处在**临时/中间状态**, 不该参与优化。
///
/// 实际见到的样子:
///   WL_CG7797_clip_20260817_nudge4.staging-7d6acb0480404e3792cf644cfc80d9f9
/// 上游生成/搬运数据时先落成 `<正式名>.staging-<uuid>`, 全部写完才改名成正式的。
/// 扫到这种目录说明它**正在被写**, 或者写了一半就中断了 —— 帧可能不全、位姿可能还没落盘。
/// 拿它去优化就是用残缺数据配约束, 而增量建图里污染过的位姿下一轮会被当成可信初值复用,
/// 错误一直往后传 (与 --max_io_fail 那条守门要防的是同一类事故)。
///
/// 判据就是**名字里有没有 '.'**: 正式的 session 名形如 <车牌>_clip_<日期>_<地点><序号>,
/// 全是字母数字和下划线, 一个点都没有。所以这条判据不会误伤正常数据。
inline bool isStagingDir(const fs::path& p) {
  return p.filename().string().find('.') != std::string::npos;
}

/// @brief 把跳过的临时目录报一遍。**必须打印** —— 否则 session 凭空少一个,
///        而清单里看不出任何原因, 只能去翻数据目录才知道发生了什么。
inline void reportStaging(const std::vector<std::string>& skipped) {
  if (skipped.empty()) return;
  printf("  跳过 %zu 个**临时状态**的目录 (名字里带 '.', 数据可能不全):\n", skipped.size());
  for (const auto& n : skipped) printf("    %s\n", n.c_str());
}

inline std::vector<fs::path> discoverSessions(
  const fs::path& root, const std::string& pose_dir, const std::string& data_mode = "clips") {
  std::vector<fs::path> out;
  std::vector<std::string> skipped;   // 临时状态的目录, 最后统一报一遍
  if (!fs::is_directory(root)) return out;

  if (data_mode == "keyframe") {
    // root 本身就是一个 session (只给了一个目录), 或者 root 下每个子目录是一个 session
    if (isKeyframeSession(root)) {
      out.push_back(root);
      return out;
    }
    for (const auto& e : fs::directory_iterator(root)) {
      if (!e.is_directory() || !isKeyframeSession(e.path())) continue;
      if (isStagingDir(e.path())) { skipped.push_back(e.path().filename().string()); continue; }
      out.push_back(e.path());
    }
    std::sort(out.begin(), out.end());
    reportStaging(skipped);
    return out;
  }

  for (const auto& e : fs::directory_iterator(root)) {
    if (!e.is_directory()) continue;
    // 临时目录在**看它有没有 clip 之前**就跳过 —— 它可能正在被写, 遍历它既没意义
    // 也可能撞上写到一半的文件
    if (isStagingDir(e.path())) { skipped.push_back(e.path().filename().string()); continue; }
    const fs::path clips_dir = fs::is_directory(e.path() / "clips") ? e.path() / "clips" : e.path();
    if (!fs::is_directory(clips_dir)) continue;

    bool has_clip = false;
    for (const auto& c : fs::directory_iterator(clips_dir)) {
      if (c.is_directory() && isClip(c.path(), pose_dir)) {
        has_clip = true;
        break;
      }
    }
    if (has_clip) out.push_back(e.path());
  }
  std::sort(out.begin(), out.end());
  reportStaging(skipped);
  return out;
}

inline bool loadSession(const fs::path& session_root, int index, SessionData& out, const LoadOptions& opt = LoadOptions()) {
  if (opt.data_mode == "keyframe") return loadKeyframeSession(session_root, index, out, opt.pose_src);
  out.index = index;
  out.name = session_root.filename().string();
  out.root = session_root;
  out.frames.clear();

  const fs::path clips_dir = fs::is_directory(session_root / "clips") ? session_root / "clips" : session_root;

  std::vector<fs::path> clip_roots;
  for (const auto& e : fs::directory_iterator(clips_dir)) {
    if (e.is_directory() && isClip(e.path(), opt.pose_dir)) clip_roots.push_back(e.path());
  }
  std::sort(clip_roots.begin(), clip_roots.end());

  if (clip_roots.empty()) {
    std::cerr << "[load] no clip under " << clips_dir << "\n";
    return false;
  }

  bool got_center = false, got_extr = false;
  Eigen::Vector2d center{0, 0};

  for (std::size_t ci = 0; ci < clip_roots.size(); ci++) {
    const fs::path& root = clip_roots[ci];
    const fs::path pose_path = root / opt.pose_dir;
    const fs::path cloud_dir = root / "sensors" / opt.lidar_dir;
    const fs::path calib_dir = root / "calibration";

    // ---- utm_center: 必须全局一致, 否则世界系不共享 ----
    try {
      std::ifstream ifs((calib_dir / "vehicle_params.json").string());
      json vp;
      ifs >> vp;
      const auto& cs = vp.at("coordinate_systems");
      Eigen::Vector2d c(cs.at("utm_center_x").get<double>(), cs.at("utm_center_y").get<double>());
      if (!got_center) {
        center = c;
        got_center = true;
      } else if ((c - center).norm() > 1e-6) {
        std::cerr << "[load] utm_center mismatch inside session " << out.name << " at " << root.filename() << "\n";
        return false;
      }
    } catch (const std::exception& ex) {
      std::cerr << "[load] vehicle_params.json parse failed at " << root << ": " << ex.what() << "\n";
      return false;
    }

    if (!got_extr) {
      // !! 即使 cloud_dir 换成 front_lidar, 这里也**不用**换外参 !!
      //   实测两份 yaml (front_lidar_2_vehicle / fuse_lidar_2_vehicle) 逐比特相同:
      //     q=(0.99995613, -0.0028113, -0.0084802, -0.0028168)  t=(2.7317, 0.0048, 1.8554)
      //   即 fuse_lidar 的点本来就表达在 **front_lidar 系**里 (ms_mapping 也是这么处理的:
      //   它 cloud_dir 用 fuse_lidar 而外参读 front_lidar)。
      //   所以别"顺手改成和 cloud_dir 同名" —— 那反而可能引入不一致。
      if (!readExtrinsics(calib_dir / "fuse_lidar_2_vehicle_extrinsics.yaml", out.T_v_l)) {
        std::cerr << "[load] cannot read fuse_lidar extrinsics at " << calib_dir << "\n";
        return false;
      }
      got_extr = true;
    }

    std::vector<fs::path> pose_files;
    for (const auto& e : fs::directory_iterator(pose_path)) {
      if (e.path().extension() == ".json") pose_files.push_back(e.path());
    }
    std::sort(pose_files.begin(), pose_files.end(), [](const fs::path& a, const fs::path& b) {
      try {
        return std::stod(a.stem().string()) < std::stod(b.stem().string());
      } catch (...) {
        return a.stem().string() < b.stem().string();
      }
    });

    for (const auto& pf : pose_files) {
      const std::string stem = pf.stem().string();
      const fs::path pcd = cloud_dir / (stem + ".pcd");
      if (!fs::exists(pcd)) continue;  // 位姿与点云一一对应, 缺一个就跳过该帧

      Frame f;
      try {
        std::ifstream ifs(pf.string());
        json j;
        ifs >> j;
        const auto& p = j.at("position");

        f.T_w_v.setIdentity();
        f.T_w_v.translation() =
          Eigen::Vector3d(p.at("x").get<double>(), p.at("y").get<double>(), p.at("z").get<double>());

        if (j.contains("euler_angles")) {
          const auto& e = j.at("euler_angles");
          f.roll = e.at("x").get<double>();
          f.pitch = e.at("y").get<double>();
          f.yaw = e.at("z").get<double>();
        }

        if (opt.attitude_from == "euler") {
          if (!j.contains("euler_angles")) {
            std::cerr << "[load] " << pf << " 缺 euler_angles\n";
            return false;
          }
          f.T_w_v.linear() = rotFromRPY(f.roll, f.pitch, f.yaw);
        } else {
          const auto& o = j.at("orientation");
          Eigen::Quaterniond q(
            o.at("qw").get<double>(),
            o.at("qx").get<double>(),
            o.at("qy").get<double>(),
            o.at("qz").get<double>());
          f.T_w_v.linear() = q.normalized().toRotationMatrix();
        }

        f.ts = j.contains("timestamp_s") ? j.at("timestamp_s").get<double>() : std::stod(stem);
      } catch (const std::exception& ex) {
        std::cerr << "[load] pose parse failed " << pf << ": " << ex.what() << "\n";
        continue;
      }

      f.T_w_l = f.T_w_v * out.T_v_l;
      f.pcd_path = pcd.string();
      f.clip_id = static_cast<int>(ci);
      out.frames.push_back(std::move(f));
    }
  }

  out.utm_center = center;
  return !out.frames.empty();
}

}  // namespace ialign

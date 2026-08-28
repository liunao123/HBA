#pragma once
// -----------------------------------------------------------------------------
// 视觉重投影约束 —— 移植 ms_mapping.cc 的 build_visual_loop_correspondences + ReprojError
//
// 核心构造 (为什么它不需要 3D 点变量):
//   帧 a 的 ORB 关键点, 借"局部子图投影到图像 a"拿到对应的 3D 点 p_a (在**帧 a 的雷达系**),
//   再把 p_a 经两个位姿变量重投影到图像 b, 与帧 b 上匹配到的像素比。
//       残差 = π( T_c_l · T_b⁻¹ · T_a · p_a ) − (u_b, v_b)      2 维
//   p_a 是常量, 所以这是一条**只连 T_a 和 T_b 的二元因子**, 不引入路标变量,
//   规模上和一条相对位姿边一样便宜, 但它约束的方向和点云配准互补。
//
// 为什么值得加 (和本项目已实测的短板对上):
//   沿路方向是激光的退化方向 —— intra_prior_xy 那轮扫描量出来: 水平放开会让缝
//   从 0.146 涨到 0.193, 因为沿路平移时最近邻代价近乎平坦, GICP 在这个方向没有约束力。
//   而车道线/杆子/路牌的像素位置对沿路平移极敏感, 正好补上。
//
// !! 必须先看这一条: 信息量量级 !!
//   本项目已经三次踩到"先验对点云因子完全惰性"的坑(见 intra_prior_xy 的注释)。
//   这里是同一个问题的第四次:
//     IntegratedVGICPFactor 的信息量**逐点累加** —— 一条因子几千到几万点, 量级 10⁴~10⁵;
//     一条重投影因子是 1/σ_px², σ=10px 时只有 0.01/维。
//   即便建出 5 万条对应, 总信息量也只有约 5×10²  —— 比单条 VGICP 因子还小。
//   所以**直接塞进 kfba 的图里, 视觉项极可能一动不动**。
//   ms_mapping 那边不存在这个问题, 因为它的 GICP 项是 TError: 一条**冻结的 6-dof 相对位姿**
//   残差, σ 手工给到 0.03m, 和重投影是同一个量级的东西。
//   处理办法不是猜权重, 而是量: visual_report 会打印视觉项占总图误差的比例, 以及
//   开/关视觉的位姿差。先看那两个数, 再决定 visual_sigma_px / visual_weight 怎么给。
//
// 与 ms_mapping 的两处有意偏离:
//   1. 深度子图用**组合导航的相对位姿**拼, 而不是链式 GICP 结果。本项目实测组合导航在
//      10m 尺度是厘米级(这也是 seq_ins 边的依据), 拼 ±3 关键帧不需要借配准结果,
//      也就不会把配准的缝引进深度里。
//   2. 因此**不做 yaw-only**。ms_mapping 丢弃 pitch/roll 是因为它链式累积会漂;
//      而本项目实测 yaw_only 在 46m 跨度上明显变坏(缝 0.172 -> 0.385)。用完整相对位姿。
// -----------------------------------------------------------------------------

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>
#include <fstream>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace ialign {

namespace vfs = std::filesystem;

// -----------------------------------------------------------------------------
// 相机模型
// -----------------------------------------------------------------------------
struct CamModel {
  std::string name;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int width = 0, height = 0;
  Eigen::Isometry3d T_c_l = Eigen::Isometry3d::Identity();  // 雷达系 -> 相机系
  // 相机->车体的原始外参。把雷达外参 T_v_l 放进 BA 一起优化时必须留着它:
  // 那时 T_c_l 要在每次迭代里现算 (T_c_l = T_v_c^-1 * T_v_l), 不能用固定值。
  Eigen::Isometry3d T_v_c = Eigen::Isometry3d::Identity();
};

/// @brief 读 OpenCV(%YAML:1.0) 内参。distCoeffs 非零会**明确报出来** ——
///        两个数据集实测都是全 0(图像已去畸变), 所以这里只用 K 做纯针孔投影。
///        若哪天换了带畸变的图, 不提示就会静默地投歪几个像素。
inline bool readIntrinsics(const vfs::path& f, CamModel& c) {
  cv::FileStorage fs(f.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) return false;
  cv::Mat K, D;
  fs["cameraMatrix"] >> K;
  fs["distCoeffs"] >> D;
  int w = 0, h = 0;
  fs["ImageWidth"] >> w;
  fs["ImageHeight"] >> h;
  fs.release();
  if (K.empty() || K.rows != 3) return false;
  c.fx = K.at<double>(0, 0);
  c.fy = K.at<double>(1, 1);
  c.cx = K.at<double>(0, 2);
  c.cy = K.at<double>(1, 2);
  c.width = w;
  c.height = h;
  if (!D.empty()) {
    double m = 0;
    for (int i = 0; i < D.rows * D.cols; i++) m = std::max(m, std::abs(D.at<double>(i)));
    if (m > 1e-9) {
      printf("  [visual] !! %s 的 distCoeffs 非零(max=%.4g), 而这里只做纯针孔投影 ——\n"
             "     两个数据集实测都是全 0(图像已去畸变)。若这批图**没有**去畸变, 重投影会\n"
             "     系统性偏几个像素, 而残差看起来只是'大了一点', 查不出来。\n",
             f.filename().c_str(), m);
    }
  }
  return c.fx > 0;
}

/// @brief 读 r_quaternion_wxyz + t_metric_xyz 外参 (与雷达外参同格式)
inline bool readQuatExtrinsic(const vfs::path& f, Eigen::Isometry3d& T) {
  cv::FileStorage fs(f.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) return false;
  cv::Mat q, t;
  fs["r_quaternion_wxyz"] >> q;
  fs["t_metric_xyz"] >> t;
  fs.release();
  if (q.empty() || t.empty()) return false;
  const Eigen::Quaterniond qq(q.at<double>(0), q.at<double>(1), q.at<double>(2), q.at<double>(3));
  T.setIdentity();
  T.linear() = qq.normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));
  return true;
}

// -----------------------------------------------------------------------------
// 相机加载 + 图像路径推导。两种数据组织方式的差别都收在这里。
//
//   clips (WG_wuling):
//     标定 <clip>/calibration/<cam>_intrinsics.yaml + <cam>_extrinsics.yaml
//     图像 <clip>/sensors/<cam>/<stem>.jpg        (与 sensors/fuse_lidar/<stem>.pcd 同 stem)
//     T_v_l 来自 session 已读到的雷达外参
//
//   keyframe (dwm_data):
//     标定 <sess>/calib/new_intrinsics/<cam>_Intrinsic.yaml
//          <sess>/calib/extrinsics/<id>_<cam>_2_vehicle_Extrinsics.yaml
//     图像 <sess>/images/<cam>/<stem>_<cam>.jpg   (stem 同 pointclouds/<stem>.pcd)
//     !! T_v_l 必须另外从 calib/extrinsics 里的**雷达**外参读 !!
//        keyframe 模式下 SessionData::T_v_l 是单位矩阵(因为 pose 本身就是雷达位姿),
//        但相机外参是相对**车体**的, 拿单位矩阵去组合会引入那条雷达外参里的
//        2.12m z 偏移 + 90 度 yaw —— 投影会整体错开, 而残差只是"偏大", 看不出是外参问题。
// -----------------------------------------------------------------------------
inline std::vector<CamModel> loadCameras(
  const vfs::path& session_root,
  const std::string& data_mode,
  const Eigen::Isometry3d& T_v_l_session,   // clips: session 的雷达外参; keyframe: 忽略
  const std::string& want_csv,              // 逗号分隔的相机名; 空 = 全部
  const std::string& sample_pcd) {          // 用于定位 clip 目录 (clips 模式)
  std::vector<CamModel> out;
  std::vector<std::string> want;
  if (!want_csv.empty()) {
    std::size_t p = 0;
    while (p <= want_csv.size()) {
      const auto c = want_csv.find(',', p);
      const std::string t = want_csv.substr(p, c == std::string::npos ? std::string::npos : c - p);
      if (!t.empty()) want.push_back(t);
      if (c == std::string::npos) break;
      p = c + 1;
    }
  }
  const auto wanted = [&](const std::string& n) {
    return want.empty() || std::find(want.begin(), want.end(), n) != want.end();
  };

  if (data_mode == "keyframe") {
    const vfs::path cal = session_root / "calib";
    const vfs::path idir = vfs::is_directory(cal / "new_intrinsics") ? cal / "new_intrinsics"
                                                                    : cal / "intrinsics";
    const vfs::path edir = cal / "extrinsics";
    if (!vfs::is_directory(idir) || !vfs::is_directory(edir)) {
      printf("  [visual] 找不到 %s 或 %s\n", idir.c_str(), edir.c_str());
      return out;
    }
    // 雷达外参: 文件名里带 lidar / hesai / velodyne 的那一个
    Eigen::Isometry3d T_v_l = Eigen::Isometry3d::Identity();
    bool got_lidar = false;
    for (const auto& e : vfs::directory_iterator(edir)) {
      const std::string n = e.path().filename().string();
      if (n.find("lidar") == std::string::npos && n.find("hesai") == std::string::npos &&
          n.find("velodyne") == std::string::npos)
        continue;
      if (n.find("cam") != std::string::npos) continue;
      if (readQuatExtrinsic(e.path(), T_v_l)) {
        got_lidar = true;
        printf("  [visual] 雷达外参 T_v_l <- %s  平移=(%.3f, %.3f, %.3f)m\n", n.c_str(),
               T_v_l.translation().x(), T_v_l.translation().y(), T_v_l.translation().z());
        break;
      }
    }
    if (!got_lidar) {
      printf("  [visual] !! calib/extrinsics 里没找到雷达外参, 无法把相机外参接到雷达系上。\n"
             "     keyframe 模式的位姿是**雷达**位姿, 而相机外参是相对车体的, 缺了这一环\n"
             "     T_c_l 就是错的(会整体偏移), 所以这里直接放弃视觉约束而不是硬算。\n");
      return out;
    }
    for (const auto& e : vfs::directory_iterator(idir)) {
      const std::string stem = e.path().stem().string();          // cam3_Intrinsic
      const auto us = stem.find('_');
      const std::string cam = us == std::string::npos ? stem : stem.substr(0, us);
      if (!wanted(cam)) continue;
      CamModel c;
      c.name = cam;
      if (!readIntrinsics(e.path(), c)) continue;
      // 找 <id>_<cam>_2_vehicle_Extrinsics.yaml
      Eigen::Isometry3d T_v_c;
      bool got = false;
      for (const auto& e2 : vfs::directory_iterator(edir)) {
        const std::string n2 = e2.path().filename().string();
        if (n2.find("_" + cam + "_") == std::string::npos) continue;
        got = readQuatExtrinsic(e2.path(), T_v_c);
        break;
      }
      if (!got) continue;
      c.T_v_c = T_v_c;
      c.T_c_l = T_v_c.inverse() * T_v_l;
      out.push_back(c);
    }
  } else {
    // clips: 标定在**某个 clip** 的 calibration/ 下 (各 clip 相同, 取样本帧所在那个)
    vfs::path cal;
    if (!sample_pcd.empty()) {
      // <clip>/sensors/fuse_lidar/<stem>.pcd -> <clip>/calibration
      cal = vfs::path(sample_pcd).parent_path().parent_path().parent_path() / "calibration";
    }
    if (!vfs::is_directory(cal)) {
      printf("  [visual] 找不到 clip 的 calibration 目录 (由 %s 推导)\n", sample_pcd.c_str());
      return out;
    }
    for (const auto& e : vfs::directory_iterator(cal)) {
      const std::string n = e.path().filename().string();
      const auto pos = n.find("_intrinsics.yaml");
      if (pos == std::string::npos) continue;
      const std::string cam = n.substr(0, pos);
      if (!wanted(cam)) continue;
      CamModel c;
      c.name = cam;
      if (!readIntrinsics(e.path(), c)) continue;
      Eigen::Isometry3d T_v_c;
      if (!readQuatExtrinsic(cal / (cam + "_extrinsics.yaml"), T_v_c)) continue;
      c.T_v_c = T_v_c;
      c.T_c_l = T_v_c.inverse() * T_v_l_session;
      out.push_back(c);
    }
  }
  std::sort(out.begin(), out.end(), [](const CamModel& a, const CamModel& b) { return a.name < b.name; });
  return out;
}

/// @brief 由点云路径推出同一时刻该相机的图像路径 (不存在则返回空)
inline vfs::path imagePathFor(const std::string& pcd_path, const std::string& cam,
                             const std::string& data_mode) {
  const vfs::path p(pcd_path);
  const std::string stem = p.stem().string();
  vfs::path img;
  if (data_mode == "keyframe") {
    // <sess>/pointclouds/<stem>.pcd -> <sess>/images/<cam>/<stem>_<cam>.jpg
    img = p.parent_path().parent_path() / "images" / cam / (stem + "_" + cam + ".jpg");
  } else {
    // <clip>/sensors/fuse_lidar/<stem>.pcd -> <clip>/sensors/<cam>/<stem>.jpg
    img = p.parent_path().parent_path() / cam / (stem + ".jpg");
  }
  return vfs::exists(img) ? img : vfs::path();
}

// -----------------------------------------------------------------------------
// 动态物体门 —— 这一条不是可选项, 缺了整个视觉约束是**有害**的
//
// 实测(WG_wuling 845,-690 那片路口, 堵车): 不加这道门时保留下来的对应几乎全落在车上 ——
// 尾灯、车牌、车身。帧 i 的 3D 点在一辆车上, 到帧 j 时那辆车已经动了, 于是重投影残差
// 编码的是**车的运动**, 不是位姿误差。
// 基础矩阵 RANSAC 拦不住: 同一辆刚体车上的匹配彼此自洽, 车占的匹配数占优时
// RANSAC 会选中"车的共识"而把静态背景判成外点 —— 越滤越错。
//
// 两批数据的动态信息形式不同, 都用上:
//   clips (WG_wuling): <clip>/annotations/3dod/v1/base_link.json
//     每个时间戳一批物体, 类别实测全是动态(car 1807 / cyclist 97 / van 89 / bicycle 15),
//     **直接带各相机的 2D 框**(mapping_objects), 还带车体系的 3D 框(dimensions+pose)。
//     -> 2D 框滤关键点, 3D 框滤深度子图的点。
//     注意 is_ego 字段实测全为 true, 不是"自车"标志, 不能拿它区分。
//   keyframe (dwm_data): masks/dynamic/<cam>/<stem>.png (300x480, 原图 1/4)
//     二值 0/255, 非零占 98.2% -> **255=保留, 0=动态**, 与 ms_mapping 的
//     `if (mask < 128) continue` 同一约定。-> 关键点和深度点都按 mask 判。
// -----------------------------------------------------------------------------
struct Obb {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();   // 中心 (车体系)
  Eigen::Matrix3d Rt = Eigen::Matrix3d::Identity();  // R_vehicle_box 的转置
  Eigen::Vector3d half = Eigen::Vector3d::Zero();
};
struct DynFrame {
  std::vector<Obb> obb;                                            // 车体系 3D 框
  std::unordered_map<std::string, std::vector<cv::Rect2f>> rect;   // 相机名 -> 2D 框
};

/// @brief clips 模式: 读一个 clip 的 3dod 标注, 按时间戳索引
inline std::map<double, DynFrame> loadDynClip(const vfs::path& clip_root) {
  std::map<double, DynFrame> out;
  const vfs::path jf = clip_root / "annotations" / "3dod" / "v1" / "base_link.json";
  if (!vfs::exists(jf)) return out;
  try {
    std::ifstream ifs(jf.string());
    nlohmann::json j;
    ifs >> j;
    if (!j.contains("objects")) return out;
    for (const auto& o : j["objects"]) {
      if (!o.contains("timestamp")) continue;
      const double ts = o["timestamp"].get<double>();
      auto& df = out[ts];
      // 3D 框
      if (o.contains("pose") && o.contains("dimensions")) {
        const auto& ps = o["pose"];
        const auto& dm = o["dimensions"];
        if (ps.contains("position") && ps.contains("quaternion")) {
          const auto& pp = ps["position"];
          const auto& qq = ps["quaternion"];
          const double L = dm.value("length", 0.0), W = dm.value("width", 0.0),
                       H = dm.value("height", 0.0);
          if (L > 0 && W > 0 && H > 0 && pp.size() >= 3 && qq.size() >= 4) {
            Obb b;
            b.c = Eigen::Vector3d(pp[0].get<double>(), pp[1].get<double>(), pp[2].get<double>());
            Eigen::Quaterniond q(qq[0].get<double>(), qq[1].get<double>(), qq[2].get<double>(),
                                 qq[3].get<double>());
            b.Rt = q.normalized().toRotationMatrix().transpose();
            b.half = Eigen::Vector3d(L * 0.5, W * 0.5, H * 0.5);
            df.obb.push_back(b);
          }
        }
      }
      // 各相机 2D 框
      if (o.contains("mapping_objects")) {
        for (const auto& m : o["mapping_objects"]) {
          if (!m.contains("camera") || !m.contains("geometry")) continue;
          const auto& gg = m["geometry"];
          const float x0 = gg.value("xmin", 0.0f), y0 = gg.value("ymin", 0.0f);
          const float x1 = gg.value("xmax", 0.0f), y1 = gg.value("ymax", 0.0f);
          if (x1 > x0 && y1 > y0) {
            df.rect[m["camera"].get<std::string>()].emplace_back(x0, y0, x1 - x0, y1 - y0);
          }
        }
      }
    }
  } catch (const std::exception& e) {
    printf("  [visual] 读 %s 失败: %s\n", jf.c_str(), e.what());
  }
  return out;
}

/// @brief 关键点/像素的屏蔽依据。两种模式二选一, 都为空就是没有动态门。
struct KpMask {
  cv::Mat mask;                     // keyframe: <128 的像素剔除 (可能是缩放过的)
  std::vector<cv::Rect2f> boxes;    // clips: 框内剔除
  double box_margin = 4.0;          // 2D 框外扩几个像素 (框边缘的点也不可靠)

  bool blocked(const cv::Point2f& p, int img_w, int img_h) const {
    for (const auto& r : boxes) {
      if (p.x >= r.x - box_margin && p.x <= r.x + r.width + box_margin &&
          p.y >= r.y - box_margin && p.y <= r.y + r.height + box_margin)
        return true;
    }
    if (!mask.empty() && img_w > 0 && img_h > 0) {
      // mask 分辨率可能低于原图 (dwm 是 1/4), 按比例索引
      const int mx = static_cast<int>(p.x * mask.cols / img_w);
      const int my = static_cast<int>(p.y * mask.rows / img_h);
      if (mx < 0 || my < 0 || mx >= mask.cols || my >= mask.rows) return true;
      if (mask.at<uchar>(my, mx) < 128) return true;
    }
    return false;
  }
  bool empty() const { return mask.empty() && boxes.empty(); }
};

/// @brief keyframe 模式: 读 masks/dynamic/<cam>/<stem>.png
inline cv::Mat loadDynMask(const std::string& pcd_path, const std::string& cam) {
  const vfs::path p(pcd_path);
  const vfs::path mp =
    p.parent_path().parent_path() / "masks" / "dynamic" / cam / (p.stem().string() + ".png");
  if (!vfs::exists(mp)) return cv::Mat();
  cv::Mat m = cv::imread(mp.string(), cv::IMREAD_GRAYSCALE);
  return m;
}

// -----------------------------------------------------------------------------
// 从**建图点云**里剔掉动态物体 (对应 ms_mapping 的 remove_box_points)
//
// 为什么必须做, 而且和 BA 是两件事:
//   实测 WG_wuling 最差的 submap (内部重影 0.198m) 出图一看: 静态结构(龙门架/杆子/路缘)
//   基本是黄色即已对齐, 而路面上是**成排的红绿配对团块** —— 车的形状和尺寸。
//   这批数据一帧 0.5 秒, 10m/s 的车两帧之间走 10 米。
//   所以"submap 内部有重影"里有相当一部分根本不是位姿误差:
//     - 奇偶两半之间: 同一辆车在偶帧和奇帧各在一个位置 -> 一对红绿;
//     - **单独一半内部**也有: 一辆车在第 0/2/4 帧各一个位置 -> A 自己就有好几个影子。
//   位姿再准也修不掉, 局部 BA 加多少轮都没用(实测 BA 生效 469/469、每帧修正 0.022m,
//   重影照旧)。要去掉只能按标注把车上的点删掉。
//
// 标注是车体系(base_link)的 3D 框, 点云在雷达系, 所以要 T_v_l。这里从**该 clip 自己的**
// calibration 读, 不依赖调用方传 —— 各 clip 理论上可以有不同标定, 而且这样调用点最干净。
// -----------------------------------------------------------------------------
struct DynClip {
  std::map<double, DynFrame> frames;
  Eigen::Isometry3d T_v_l = Eigen::Isometry3d::Identity();
  bool tried = false, ok = false;
};

inline DynClip& dynClipFor(const vfs::path& clip_root) {
  static std::map<std::string, DynClip> cache;
  static std::mutex mu;
  std::lock_guard<std::mutex> lk(mu);
  auto& e = cache[clip_root.string()];
  if (e.tried) return e;
  e.tried = true;
  e.frames = loadDynClip(clip_root);
  // 雷达外参: <clip>/calibration/*lidar*_2_vehicle_extrinsics.yaml
  // !! 必须**精确**匹配文件名, 不能用 find("front") !!
  //   目录里还有 right_front_lidar_2_vehicle_extrinsics.yaml, 它也含 "front", 而它的外参
  //   差 0.68m (t.y = -0.671 vs 0.005)。目录遍历顺序是未定义的, 之前那版靠 front_lidar
  //   恰好排在前面才侥幸正确 —— 换个文件系统就会静默地把框放错 0.68m。
  //   (fuse_lidar 与 front_lidar 的外参逐比特相同, 所以两者都可用, 按此顺序取。)
  const vfs::path cal = clip_root / "calibration";
  for (const char* nm : {"fuse_lidar_2_vehicle_extrinsics.yaml",
                         "front_lidar_2_vehicle_extrinsics.yaml"}) {
    if (readQuatExtrinsic(cal / nm, e.T_v_l)) break;
  }
  e.ok = !e.frames.empty();
  return e;
}

/// @brief 就地删掉落在该帧动态框内的点。返回删掉的点数; <0 表示这一帧没有可用标注。
inline long removeDynPoints(const std::string& pcd_path, double ts_tol,
                           std::vector<Eigen::Vector4d>& pts, std::vector<double>& ints) {
  const vfs::path p(pcd_path);
  // <clip>/sensors/fuse_lidar/<ts>.pcd -> <clip>
  const vfs::path clip = p.parent_path().parent_path().parent_path();
  auto& dc = dynClipFor(clip);
  if (!dc.ok) return -1;
  double ts = 0;
  try {
    ts = std::stod(p.stem().string());
  } catch (...) {
    return -1;
  }
  // 最近时间戳
  const DynFrame* df = nullptr;
  double bd = 1e18;
  auto it = dc.frames.lower_bound(ts);
  for (int d = -1; d <= 1; d++) {
    auto q = it;
    if (d < 0) {
      if (q == dc.frames.begin()) continue;
      --q;
    } else if (d > 0) {
      if (q == dc.frames.end()) continue;
      ++q;
      if (q == dc.frames.end()) continue;
    } else if (q == dc.frames.end()) {
      continue;
    }
    const double dd = std::abs(q->first - ts);
    if (dd < bd) {
      bd = dd;
      df = &q->second;
    }
  }
  if (!df || bd > ts_tol || df->obb.empty()) return -1;

  const bool hi = ints.size() == pts.size();
  std::vector<Eigen::Vector4d> kp;
  std::vector<double> ki;
  kp.reserve(pts.size());
  if (hi) ki.reserve(ints.size());
  for (std::size_t i = 0; i < pts.size(); i++) {
    const Eigen::Vector3d p_v = dc.T_v_l * pts[i].head<3>();
    bool in = false;
    for (const auto& b : df->obb) {
      const Eigen::Vector3d q = b.Rt * (p_v - b.c);
      // 与 ms_mapping 一致的外扩: 框标注偏紧, 边缘点仍属于车
      if (std::abs(q.x()) <= b.half.x() + 0.3 && std::abs(q.y()) <= b.half.y() + 0.2 &&
          std::abs(q.z()) <= b.half.z() + 0.1) {
        in = true;
        break;
      }
    }
    if (in) continue;
    kp.push_back(pts[i]);
    if (hi) ki.push_back(ints[i]);
  }
  const long removed = static_cast<long>(pts.size() - kp.size());
  pts.swap(kp);
  if (hi) ints.swap(ki);
  return removed;
}

// -----------------------------------------------------------------------------
// ORB 特征 (含 ms_mapping 的三道筛)
// -----------------------------------------------------------------------------
struct VisualOpts {
  bool enable = false;
  std::string cams;             // 逗号分隔, 空 = 全部
  int orb_features = 1500;      // 每图最终保留数
  int grid_cell_px = 60;        // 网格去簇: 格子边长
  int max_per_cell = 4;         // 每格最多保留
  double max_desc_dist = 64.0;  // ORB 汉明距离上限
  double fmat_px = 2.0;         // 基础矩阵 RANSAC 对极距离
  int min_fmat_pts = 8;         // 少于此数无法验证极几何 -> 整条边丢弃
  int max_corr_per_edge = 50;   // 每条边最多保留多少对应
  double nearest_px = 5.0;      // 关键点到投影点的最大像素距离
  double gate_px = 10.0;        // 用当前位姿预筛: 重投影误差上限
  int submap_half = 3;          // 深度子图取 ±N 个关键帧
  double submap_voxel = 0.10;   // 深度子图体素
  double submap_max_dist = 10.0;// 邻帧离中心超过这个距离就不进子图
  double sigma_px = 10.0;       // 重投影 sigma
  double weight = 1.0;          // 信息矩阵额外放大 (对抗点云因子的量级差, 见文件头)
  double cauchy = 1.0;          // Cauchy 鲁棒核参数; <=0 关闭
  bool use_dynamic = true;      // 用动态物体标注/mask 屏蔽 (见 KpMask 上方注释, 关掉会有害)
  double ts_tol = 0.02;         // clips: 帧时间戳与标注时间戳的匹配容差 (s)
  int dump_max = 0;             // 出多少组调试图
  int dump_stride = 10;         // 每隔多少条边出一组
};

struct FeatSet {
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;
};

/// @brief 关键点邻域是不是天空/无纹理 (邻域近似同色, 或整体偏白/低饱和)。
///        这类点即便匹配上了, 在 LiDAR 投影里也大概率没有回波, 容易产生错匹配。
inline bool lowTextureKp(const cv::Mat& bgr, const cv::Point2f& pt, int radius = 4,
                        double std_thr = 4.0, double bright_thr = 235.0, double sat_thr = 12.0) {
  const int cx = static_cast<int>(std::lround(pt.x)), cy = static_cast<int>(std::lround(pt.y));
  const int x0 = std::max(0, cx - radius), x1 = std::min(bgr.cols - 1, cx + radius);
  const int y0 = std::max(0, cy - radius), y1 = std::min(bgr.rows - 1, cy + radius);
  if (x1 <= x0 || y1 <= y0) return true;
  const cv::Mat patch = bgr(cv::Range(y0, y1 + 1), cv::Range(x0, x1 + 1));
  cv::Mat gray, hsv;
  cv::cvtColor(patch, gray, cv::COLOR_BGR2GRAY);
  cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);
  cv::Scalar mg, sg, mh, sh;
  cv::meanStdDev(gray, mg, sg);
  cv::meanStdDev(hsv, mh, sh);
  return sg[0] < std_thr || (mg[0] > bright_thr && mh[1] < sat_thr);
}

/// @brief 提特征: ORB -> 低纹理剔除 -> 网格去簇 -> 按 response 截断
inline bool extractFeat(const vfs::path& img_path, const VisualOpts& o, const KpMask& km,
                        FeatSet& out) {
  const cv::Mat img = cv::imread(img_path.string());
  if (img.empty()) return false;
  // 超采样再筛: ORB 内部按 response 全局截点时, 弱纹理区域的点会先被强纹理区域挤掉,
  // 多取 4 倍保证网格化后每格仍有候选。
  auto orb = cv::ORB::create(o.orb_features * 4);
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;
  orb->detectAndCompute(img, cv::noArray(), kps, desc);
  if (kps.empty()) return false;

  FeatSet f;
  for (int k = 0; k < static_cast<int>(kps.size()); k++) {
    if (lowTextureKp(img, kps[k].pt)) continue;
    if (km.blocked(kps[k].pt, img.cols, img.rows)) continue;   // 动态物体 (见 KpMask 上方注释)
    f.kps.push_back(kps[k]);
    f.desc.push_back(desc.row(k));
  }
  if (f.kps.empty()) return false;

  // 网格去簇: 避免树叶/灌木这类高纹理区独占整张图的特征额度
  std::unordered_map<long long, std::vector<int>> buckets;
  for (int k = 0; k < static_cast<int>(f.kps.size()); k++) {
    const long long gx = static_cast<long long>(f.kps[k].pt.x / o.grid_cell_px);
    const long long gy = static_cast<long long>(f.kps[k].pt.y / o.grid_cell_px);
    buckets[(gx << 32) | (gy & 0xffffffffLL)].push_back(k);
  }
  std::vector<char> keep(f.kps.size(), 0);
  for (auto& [key, idx] : buckets) {
    std::sort(idx.begin(), idx.end(),
              [&](int a, int b) { return f.kps[a].response > f.kps[b].response; });
    for (int n = 0; n < std::min<int>(o.max_per_cell, static_cast<int>(idx.size())); n++) keep[idx[n]] = 1;
  }
  std::vector<int> sel;
  for (int k = 0; k < static_cast<int>(f.kps.size()); k++) {
    if (keep[k]) sel.push_back(k);
  }
  if (static_cast<int>(sel.size()) > o.orb_features) {
    std::sort(sel.begin(), sel.end(),
              [&](int a, int b) { return f.kps[a].response > f.kps[b].response; });
    sel.resize(o.orb_features);
  }
  out.kps.clear();
  out.desc.release();
  for (const int k : sel) {
    out.kps.push_back(f.kps[k]);
    out.desc.push_back(f.desc.row(k));
  }
  return !out.kps.empty();
}

// -----------------------------------------------------------------------------
// 一条视觉对应
// -----------------------------------------------------------------------------
struct VisualCorr {
  int i = -1, j = -1;                        // 关键帧下标 (i 提供 3D 点, j 提供像素)
  int cam = 0;                               // CamModel 下标
  Eigen::Vector3d p_i = Eigen::Vector3d::Zero();  // 帧 i 的雷达系下的 3D 点
  double u = 0, v = 0;                       // 图像 j 上的观测像素
  double gate_err = -1;                      // 建边时用当前位姿算的重投影误差 (px)
};

struct VisualStats {
  long edges = 0, edges_no_feat = 0, edges_no_depth = 0, edges_few_fmat = 0;
  long raw = 0, drop_desc = 0, drop_fmat = 0, drop_cap = 0, drop_nodepth = 0, drop_gate = 0, kept = 0;
  double err_sum = 0;
};

// -----------------------------------------------------------------------------
// gtsam 因子: 残差 = π(T_c_l · T_b⁻¹ · T_a · p_a) − uv
//
// Jacobian 用数值微分。理由: 这里的因子数量(万级, 2 维)在总代价里微不足道 ——
// 同图里的 VGICP 因子每条要遍历几千到几万个点。省这点解析求导的代价, 换来
// 不会因为手推 Jacobian 出错而得到一个"看起来收敛了但方向不对"的结果。
// -----------------------------------------------------------------------------
class ReprojFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
  Eigen::Vector3d p_a_;
  Eigen::Matrix4d M_c_l_;
  double fx_, fy_, cx_, cy_;
  gtsam::Vector2 uv_;
  double z_min_;

 public:
  ReprojFactor(gtsam::Key ka, gtsam::Key kb, const Eigen::Vector3d& p_a,
               const Eigen::Isometry3d& T_c_l, double fx, double fy, double cx, double cy, double u,
               double v, const gtsam::SharedNoiseModel& nm, double z_min = 0.5)
    : Base(nm, ka, kb),
      p_a_(p_a),
      M_c_l_(T_c_l.matrix()),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      uv_(u, v),
      z_min_(z_min) {}

  gtsam::Vector2 residual(const gtsam::Pose3& Ta, const gtsam::Pose3& Tb) const {
    const Eigen::Vector4d ph(p_a_.x(), p_a_.y(), p_a_.z(), 1.0);
    const Eigen::Vector4d pc = M_c_l_ * (Tb.matrix().inverse() * (Ta.matrix() * ph));
    // z 夹住而不是让因子失效: 建边时已用当前位姿筛过(gate_px), 所以 z<=0 实际不会发生;
    // 真发生了也给一个有界的大残差交给鲁棒核压住, 而不是返回 0 让优化器以为误差降了。
    const double z = std::max(z_min_, pc.z());
    return gtsam::Vector2(fx_ * pc.x() / z + cx_, fy_ * pc.y() / z + cy_) - uv_;
  }

  // !! 签名必须跟着 gtsam 版本走 !!
  //   本项目实际编译用的是源码树 /mnt/nvme0n1p2/gtsam_map/gtsam (4.3a0), 不是
  //   /usr/local/include/gtsam (4.1.1) —— 后者是另一份, 只是恰好也装在系统里。
  //   4.1 是 boost::optional<Matrix&>, 4.3 换成了 OptionalMatrixType = Matrix* (裸指针)。
  //   写错了编译器只会说"marked override, but does not override", 不会告诉你差在哪。
  gtsam::Vector evaluateError(const gtsam::Pose3& Ta, const gtsam::Pose3& Tb,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override {
    if (H1 || H2) {
      const std::function<gtsam::Vector2(const gtsam::Pose3&, const gtsam::Pose3&)> h =
        [this](const gtsam::Pose3& a, const gtsam::Pose3& b) { return residual(a, b); };
      if (H1) *H1 = gtsam::numericalDerivative21<gtsam::Vector2, gtsam::Pose3, gtsam::Pose3>(h, Ta, Tb);
      if (H2) *H2 = gtsam::numericalDerivative22<gtsam::Vector2, gtsam::Pose3, gtsam::Pose3>(h, Ta, Tb);
    }
    return residual(Ta, Tb);
  }
};


// -----------------------------------------------------------------------------
// 建对应关系
//
// 组织方式: 外层按相机, 内层按"提供 3D 点的那一帧 i"分组并行。
//   为什么按相机分外层: 特征要预提取才能并行匹配, 而一个相机 2500 帧的特征约 180MB,
//   七个相机同时留在内存就是 1.2GB。按相机做完一轮就释放, 峰值只占一个相机的量。
//   为什么按 i 分组: 帧 i 的"深度子图投影"是这里最贵的一步(拼 ±3 帧 + 投影 + 建像素网格),
//   同一个 i 的所有配对共用一份, 分组后每个线程同时只持有一份投影缓存。
// -----------------------------------------------------------------------------
inline std::vector<VisualCorr> buildVisualCorrs(
  const std::vector<std::pair<int, int>>& pairs,
  const std::vector<Eigen::Isometry3d>& poses,      // 当前位姿 (只用于 gate 预筛)
  const std::vector<Eigen::Isometry3d>& ins_poses,  // 组合导航位姿 (拼深度子图, 见文件头偏离说明)
  const std::vector<CamModel>& cams,
  const std::function<vfs::path(int, int)>& img_of,               // (帧下标, 相机下标) -> 图像路径
  const std::function<std::vector<Eigen::Vector4d>(int)>& cloud_of,  // 帧下标 -> 该帧雷达系点云
  const std::function<std::string(int)>& pcd_of,                 // 帧下标 -> 点云路径 (定位标注/mask)
  const std::string& data_mode, const Eigen::Isometry3d& T_v_l,  // T_v_l: 3D 框(车体系)->雷达系
  const VisualOpts& o, int nthreads, VisualStats& st, const vfs::path& dump_dir) {
  std::vector<VisualCorr> out;
  if (cams.empty() || pairs.empty()) return out;

  // i -> 该帧参与的所有 j
  std::map<int, std::vector<int>> by_i;
  for (const auto& [i, j] : pairs) by_i[i].push_back(j);
  std::vector<int> ivec;
  for (const auto& kv : by_i) ivec.push_back(kv.first);

  // ---- 预建动态门 ----
  // clips: 标注按 clip 存(一个 json 覆盖该 clip 全部时间戳), 所以按 clip_root 缓存,
  //        一次解析全帧受益。单线程建表, 之后只读 -> 并行安全。
  std::map<std::string, std::map<double, DynFrame>> dyn_db;
  std::vector<const DynFrame*> dyn_of(poses.size(), nullptr);
  const Eigen::Matrix3d R_l_v = T_v_l.linear().transpose();          // 车体系 -> 雷达系
  const Eigen::Vector3d t_l_v = -R_l_v * T_v_l.translation();
  long n_dyn_frames = 0, n_dyn_obj = 0;
  if (o.use_dynamic && data_mode != "keyframe") {
    std::vector<char> mark(poses.size(), 0);
    for (const auto& [i, j] : pairs) {
      mark[i] = 1;
      mark[j] = 1;
    }
    for (std::size_t k = 0; k < mark.size(); k++) {
      if (!mark[k]) continue;
      const std::string pp = pcd_of(static_cast<int>(k));
      if (pp.empty()) continue;
      const vfs::path clip = vfs::path(pp).parent_path().parent_path().parent_path();
      if (!dyn_db.count(clip.string())) dyn_db[clip.string()] = loadDynClip(clip);
      const auto& m = dyn_db[clip.string()];
      if (m.empty()) continue;
      double ts = 0;
      try {
        ts = std::stod(vfs::path(pp).stem().string());
      } catch (...) {
        continue;
      }
      auto it = m.lower_bound(ts);
      const DynFrame* best = nullptr;
      double bd = 1e18;
      for (int d = -1; d <= 1; d++) {
        auto q = it;
        if (d < 0) {
          if (q == m.begin()) continue;
          --q;
        } else if (d > 0) {
          if (q == m.end()) continue;
          ++q;
          if (q == m.end()) continue;
        } else if (q == m.end()) {
          continue;
        }
        const double dd = std::abs(q->first - ts);
        if (dd < bd) {
          bd = dd;
          best = &q->second;
        }
      }
      if (best && bd <= o.ts_tol) {
        dyn_of[k] = best;
        n_dyn_frames++;
        n_dyn_obj += static_cast<long>(best->obb.size());
      }
    }
    printf("  [visual] 动态门(3dod 标注): 匹配上标注的帧=%ld  3D框合计=%ld\n", n_dyn_frames, n_dyn_obj);
    if (n_dyn_frames == 0) {
      printf("    !! 一帧都没匹配上标注 —— 视觉对应会大量落在车上(实测堵车路口几乎全在车上),\n"
             "       那种残差编码的是**车的运动**而不是位姿误差, 加进图里是有害的。\n"
             "       先查 annotations/3dod/v1/base_link.json 是否存在、时间戳是否对得上。\n");
    }
  }
  /// 点是否落在该帧的动态 3D 框内 (输入点在**雷达系**)
  const auto ptBlocked = [&](int fr, const Eigen::Vector3d& p_l) {
    const DynFrame* df = (fr >= 0 && fr < static_cast<int>(dyn_of.size())) ? dyn_of[fr] : nullptr;
    if (!df) return false;
    const Eigen::Vector3d p_v = T_v_l * p_l;   // 框在车体系, 把点转过去比
    for (const auto& b : df->obb) {
      const Eigen::Vector3d q = b.Rt * (p_v - b.c);
      // 与 ms_mapping 一致的外扩: 框标注偏紧, 边缘点仍属于车
      if (std::abs(q.x()) <= b.half.x() + 0.3 && std::abs(q.y()) <= b.half.y() + 0.2 &&
          std::abs(q.z()) <= b.half.z() + 0.1)
        return true;
    }
    return false;
  };

  std::mutex mtx;
  std::atomic<int> n_dumped{0};
  if (o.dump_max > 0) vfs::create_directories(dump_dir);

  for (int ci = 0; ci < static_cast<int>(cams.size()); ci++) {
    const auto& cam = cams[ci];

    // ---- 预提特征 (这一相机涉及的所有帧) ----
    std::vector<int> need;
    {
      std::vector<char> mark(poses.size(), 0);
      for (const auto& [i, j] : pairs) {
        mark[i] = 1;
        mark[j] = 1;
      }
      for (std::size_t k = 0; k < mark.size(); k++) {
        if (mark[k]) need.push_back(static_cast<int>(k));
      }
    }
    std::unordered_map<int, FeatSet> feat;
    feat.reserve(need.size());
    {
      std::vector<FeatSet> tmp(need.size());
      std::vector<char> ok(need.size(), 0);
#pragma omp parallel for num_threads(nthreads) schedule(dynamic)
      for (std::int64_t t = 0; t < static_cast<std::int64_t>(need.size()); t++) {
        const auto ip = img_of(need[t], ci);
        if (ip.empty()) continue;
        KpMask km;
        if (o.use_dynamic) {
          if (data_mode == "keyframe") km.mask = loadDynMask(pcd_of(need[t]), cam.name);
          else if (dyn_of[need[t]]) {
            auto it2 = dyn_of[need[t]]->rect.find(cam.name);
            if (it2 != dyn_of[need[t]]->rect.end()) km.boxes = it2->second;
          }
        }
        ok[t] = extractFeat(ip, o, km, tmp[t]) ? 1 : 0;
      }
      for (std::size_t t = 0; t < need.size(); t++) {
        if (ok[t]) feat.emplace(need[t], std::move(tmp[t]));
      }
    }
    std::size_t nk = 0;
    for (const auto& kv : feat) nk += kv.second.kps.size();
    printf("  [visual] %s: 提到特征的帧=%zu/%zu  关键点中位≈%zu\n", cam.name.c_str(), feat.size(),
           need.size(), feat.empty() ? 0 : nk / feat.size());
    fflush(stdout);
    if (feat.size() < 2) continue;

    // ---- 按 i 分组并行 ----
#pragma omp parallel for num_threads(nthreads) schedule(dynamic)
    for (std::int64_t t = 0; t < static_cast<std::int64_t>(ivec.size()); t++) {
      const int i = ivec[t];
      auto fi = feat.find(i);
      if (fi == feat.end()) continue;

      // ---- 帧 i 的深度子图 -> 投影到图像 i ----
      // 用组合导航的相对位姿把 ±submap_half 个关键帧拼到 i 的雷达系。
      std::vector<Eigen::Vector4d> sm;
      for (int d = -o.submap_half; d <= o.submap_half; d++) {
        const int k = i + d;
        if (k < 0 || k >= static_cast<int>(ins_poses.size())) continue;
        const Eigen::Isometry3d rel = ins_poses[i].inverse() * ins_poses[k];
        // 帧号连续不代表空间连续(出框再进框、掉头、缓行): 超过这个距离就不进子图
        if (rel.translation().norm() > o.submap_max_dist) continue;
        // **在帧 k 自己的时刻**判它自己的框 —— 车在动, 用帧 i 的框去判帧 k 的点是错的
        for (const auto& p : cloud_of(k)) {
          if (o.use_dynamic && ptBlocked(k, p.head<3>())) continue;
          sm.push_back(rel * p);
        }
      }
      if (sm.size() < 500) {
        std::lock_guard<std::mutex> lk(mtx);
        st.edges_no_depth += static_cast<long>(by_i.at(i).size());
        continue;
      }
      // 体素下采样 (投影只需要稀疏一点的深度锚点, 不下采样会让像素网格里挤满同一表面的点)
      {
        std::map<std::int64_t, Eigen::Vector4d> g;
        const double inv = 1.0 / o.submap_voxel;
        for (const auto& p : sm) {
          const std::int64_t cx = static_cast<std::int64_t>(std::floor(p.x() * inv));
          const std::int64_t cy = static_cast<std::int64_t>(std::floor(p.y() * inv));
          const std::int64_t cz = static_cast<std::int64_t>(std::floor(p.z() * inv));
          g.emplace(((cx & 0x1FFFFF) << 42) | ((cy & 0x1FFFFF) << 21) | (cz & 0x1FFFFF), p);
        }
        sm.clear();
        for (const auto& kv : g) sm.push_back(kv.second);
      }

      std::vector<cv::Point2f> px;
      std::vector<Eigen::Vector3d> p3;
      px.reserve(sm.size());
      p3.reserve(sm.size());
      for (const auto& p : sm) {
        const Eigen::Vector4d pc = cam.T_c_l.matrix() * p;
        if (pc.z() <= 0.1) continue;
        const double u = cam.fx * pc.x() / pc.z() + cam.cx;
        const double v = cam.fy * pc.y() / pc.z() + cam.cy;
        if (u < 0 || v < 0 || u >= cam.width || v >= cam.height) continue;
        px.emplace_back(static_cast<float>(u), static_cast<float>(v));
        p3.push_back(p.head<3>());   // 存**雷达系**的点(因子里要用它, 不是相机系)
      }
      if (px.size() < 200) {
        std::lock_guard<std::mutex> lk(mtx);
        st.edges_no_depth += static_cast<long>(by_i.at(i).size());
        continue;
      }
      // 像素网格, 加速"关键点最近的投影点"查询
      std::unordered_map<long long, std::vector<int>> pgrid;
      for (int k = 0; k < static_cast<int>(px.size()); k++) {
        const long long gx = static_cast<long long>(std::floor(px[k].x / o.nearest_px));
        const long long gy = static_cast<long long>(std::floor(px[k].y / o.nearest_px));
        pgrid[(gx << 32) | (gy & 0xffffffffLL)].push_back(k);
      }
      const auto nearest3d = [&](const cv::Point2f& q, Eigen::Vector3d& p) -> bool {
        const long long qx = static_cast<long long>(std::floor(q.x / o.nearest_px));
        const long long qy = static_cast<long long>(std::floor(q.y / o.nearest_px));
        double best = o.nearest_px * o.nearest_px;
        int bi = -1;
        for (long long dx = -1; dx <= 1; dx++) {
          for (long long dy = -1; dy <= 1; dy++) {
            auto it = pgrid.find(((qx + dx) << 32) | ((qy + dy) & 0xffffffffLL));
            if (it == pgrid.end()) continue;
            for (const int k : it->second) {
              const double d2 = (px[k].x - q.x) * (px[k].x - q.x) + (px[k].y - q.y) * (px[k].y - q.y);
              if (d2 < best) {
                best = d2;
                bi = k;
              }
            }
          }
        }
        if (bi < 0) return false;
        p = p3[bi];
        return true;
      };

      cv::BFMatcher matcher(cv::NORM_HAMMING, /*crossCheck=*/true);
      std::vector<VisualCorr> local;
      VisualStats ls;

      for (const int j : by_i.at(i)) {
        auto fj = feat.find(j);
        if (fj == feat.end()) {
          ls.edges_no_feat++;
          continue;
        }
        ls.edges++;
        std::vector<cv::DMatch> raw;
        matcher.match(fi->second.desc, fj->second.desc, raw);
        ls.raw += static_cast<long>(raw.size());

        // 1) 描述子距离
        std::vector<cv::DMatch> md;
        for (const auto& m : raw) {
          if (m.distance <= o.max_desc_dist) md.push_back(m);
        }
        ls.drop_desc += static_cast<long>(raw.size() - md.size());

        // 2) 基础矩阵 RANSAC —— 纯 2D-2D, **不依赖当前位姿**, 与下面的 gate 互补:
        //    gate 用位姿筛(位姿错了就会连正确匹配一起筛掉), 极几何只看图像本身。
        std::vector<cv::DMatch> ms;
        if (static_cast<int>(md.size()) >= o.min_fmat_pts) {
          std::vector<cv::Point2f> pa, pb;
          for (const auto& m : md) {
            pa.push_back(fi->second.kps[m.queryIdx].pt);
            pb.push_back(fj->second.kps[m.trainIdx].pt);
          }
          std::vector<uchar> inl;
          cv::findFundamentalMat(pa, pb, cv::FM_RANSAC, o.fmat_px, 0.99, inl);
          for (std::size_t k = 0; k < md.size() && k < inl.size(); k++) {
            if (inl[k]) ms.push_back(md[k]);
          }
          ls.drop_fmat += static_cast<long>(md.size() - ms.size());
        } else {
          ls.drop_fmat += static_cast<long>(md.size());
          ls.edges_few_fmat++;
        }

        // 3) 每边配额 (均匀跨步抽样, 保留空间分布)
        if (static_cast<int>(ms.size()) > o.max_corr_per_edge) {
          std::vector<cv::DMatch> cap;
          const double stride = static_cast<double>(ms.size()) / o.max_corr_per_edge;
          for (int s = 0; s < o.max_corr_per_edge; s++) {
            cap.push_back(ms[static_cast<std::size_t>(s * stride)]);
          }
          ls.drop_cap += static_cast<long>(ms.size() - cap.size());
          ms.swap(cap);
        }

        const bool want_dump = o.dump_max > 0 && n_dumped.load() < o.dump_max &&
                               (t % std::max(1, o.dump_stride) == 0);
        std::vector<cv::Point2f> dbg_obs, dbg_pred;
        std::vector<char> dbg_ok;

        for (const auto& m : ms) {
          Eigen::Vector3d p_i;
          if (!nearest3d(fi->second.kps[m.queryIdx].pt, p_i)) {
            ls.drop_nodepth++;
            continue;
          }
          const cv::Point2f& kb = fj->second.kps[m.trainIdx].pt;
          // gate: 用当前位姿算一次重投影误差
          const Eigen::Vector4d ph(p_i.x(), p_i.y(), p_i.z(), 1.0);
          const Eigen::Vector4d pc =
            cam.T_c_l.matrix() * (poses[j].inverse().matrix() * (poses[i].matrix() * ph));
          if (pc.z() <= 1e-3) {
            ls.drop_gate++;
            continue;
          }
          const double up = cam.fx * pc.x() / pc.z() + cam.cx;
          const double vp = cam.fy * pc.y() / pc.z() + cam.cy;
          const double err = std::hypot(up - kb.x, vp - kb.y);
          if (want_dump) {
            dbg_obs.push_back(kb);
            dbg_pred.emplace_back(static_cast<float>(up), static_cast<float>(vp));
            dbg_ok.push_back(err <= o.gate_px ? 1 : 0);
          }
          if (err > o.gate_px) {
            ls.drop_gate++;
            continue;
          }
          VisualCorr vc;
          vc.i = i;
          vc.j = j;
          vc.cam = ci;
          vc.p_i = p_i;
          vc.u = kb.x;
          vc.v = kb.y;
          vc.gate_err = err;
          local.push_back(vc);
          ls.kept++;
          ls.err_sum += err;
        }

        // ---- 调试图: 匹配连线 + 重投影叠加 ----
        if (want_dump && !dbg_obs.empty()) {
          const int seq = n_dumped++;
          if (seq < o.dump_max) {
            const cv::Mat ia = cv::imread(img_of(i, ci).string());
            const cv::Mat ib = cv::imread(img_of(j, ci).string());
            if (!ia.empty() && !ib.empty()) {
              char tag[160];
              std::snprintf(tag, sizeof(tag), "%s_%04d-%04d_n%03zu", cam.name.c_str(), i, j,
                            dbg_obs.size());
              cv::Mat vm;
              cv::drawMatches(ia, fi->second.kps, ib, fj->second.kps, ms, vm);
              cv::imwrite((dump_dir / (std::string(tag) + "_matches.jpg")).string(), vm);
              cv::Mat vr = ib.clone();
              double es = 0;
              int en = 0;
              for (std::size_t k = 0; k < dbg_obs.size(); k++) {
                const cv::Scalar col = dbg_ok[k] ? cv::Scalar(0, 0, 255) : cv::Scalar(160, 160, 160);
                cv::drawMarker(vr, dbg_obs[k], cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
                cv::circle(vr, dbg_pred[k], 4, col, -1, cv::LINE_AA);
                cv::line(vr, dbg_obs[k], dbg_pred[k], col, 1, cv::LINE_AA);
                if (dbg_ok[k]) {
                  es += std::hypot(dbg_obs[k].x - dbg_pred[k].x, dbg_obs[k].y - dbg_pred[k].y);
                  en++;
                }
              }
              char lbl[220];
              std::snprintf(lbl, sizeof(lbl),
                            "i=%d j=%d cand=%zu kept=%d mean_err=%.2fpx | GREEN=obs RED=pred(kept) "
                            "GRAY=pred(rejected)",
                            i, j, dbg_obs.size(), en, en ? es / en : -1.0);
              cv::putText(vr, lbl, cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
              cv::imwrite((dump_dir / (std::string(tag) + "_reproj.jpg")).string(), vr);
            }
          }
        }
      }

      std::lock_guard<std::mutex> lk(mtx);
      out.insert(out.end(), local.begin(), local.end());
      st.edges += ls.edges;
      st.edges_no_feat += ls.edges_no_feat;
      st.edges_few_fmat += ls.edges_few_fmat;
      st.raw += ls.raw;
      st.drop_desc += ls.drop_desc;
      st.drop_fmat += ls.drop_fmat;
      st.drop_cap += ls.drop_cap;
      st.drop_nodepth += ls.drop_nodepth;
      st.drop_gate += ls.drop_gate;
      st.kept += ls.kept;
      st.err_sum += ls.err_sum;
    }
  }
  return out;
}


// -----------------------------------------------------------------------------
// 按测量到的**可重复性**给点云因子重新定权
//
// 为什么不能靠白化 rms 来削弱点云:
//   白化 rms 只检验边际一致性。实测点云的白化 rms = 0.284, 即残差比它假设的逐点噪声还小
//   3.5 倍 —— 按 rms->1 校准是把它**加强** 12 倍。边际统计推不出"该削弱"。
//   点云的问题不在单条残差, 在**把一帧里几万个高度相关的点当成独立测量**;
//   相关性是边际统计看不见的。
//
// 能量它的东西: **配准的可重复性**。本项目独立测到过多次 —— submap 间配准噪声底约
//   0.197m, 单次配准噪声地板 0.203m。而因子的 Hessian 隐含了它自己声称的相对位姿 sigma。
//   两者之比的平方, 就是它高估置信度的倍数。这是算出来的, 不是拍的。
//
// 做法: 把因子的信息矩阵乘 s = (sigma_隐含 / sigma_可重复)^2, 使校准后的 sigma 等于实测
//   可重复性。s < 1 即削弱。HessianFactor 的 cost 对 (G, g, f) 整体是线性的, 所以整块
//   增广信息矩阵乘 s 即可。
// -----------------------------------------------------------------------------
class ScaledFactor : public gtsam::NonlinearFactor {
  gtsam::NonlinearFactor::shared_ptr base_;
  double s_ = 1.0;

 public:
  ScaledFactor(const gtsam::NonlinearFactor::shared_ptr& base, double s)
    : gtsam::NonlinearFactor(base->keys()), base_(base), s_(s) {}

  double error(const gtsam::Values& v) const override { return s_ * base_->error(v); }
  size_t dim() const override { return base_->dim(); }

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& v) const override {
    auto gf = base_->linearize(v);
    auto hf = std::dynamic_pointer_cast<gtsam::HessianFactor>(gf);
    if (!hf || s_ == 1.0) return gf;
    const gtsam::Matrix aug = hf->augmentedInformation() * s_;
    std::vector<size_t> dims;
    for (std::size_t k = 0; k < hf->keys().size(); k++) dims.push_back(6);   // Pose3
    dims.push_back(1);   // 增广列
    return std::make_shared<gtsam::HessianFactor>(
      hf->keys(), gtsam::SymmetricBlockMatrix(dims, aug));
  }
  double scale() const { return s_; }
};

/// @brief 从线性化后的 Hessian 反解该因子声称的**平移** sigma (m)。
///        取 gtsam Pose3 切空间顺序 (rx,ry,rz,tx,ty,tz) 的后三维块, 求逆后取对角均值开方。
///        取不到就返回 <0。
inline double impliedTransSigma(const gtsam::NonlinearFactor::shared_ptr& f,
                               const gtsam::Values& v) {
  auto gf = f->linearize(v);
  if (!gf) return -1.0;
  const gtsam::Matrix I = gf->information();
  // 只有一个 key 时 I 是 6x6; 两个 key 时是 12x12, 取第一个块(对相对位姿等价)
  if (I.rows() < 6 || I.cols() < 6) return -1.0;
  const Eigen::Matrix3d Itt = I.block<3, 3>(3, 3);
  if (Itt.determinant() <= 1e-18) return -1.0;
  const Eigen::Matrix3d C = Itt.inverse();
  const double m = (C(0, 0) + C(1, 1) + C(2, 2)) / 3.0;
  return m > 0 ? std::sqrt(m) : -1.0;
}

}  // namespace ialign

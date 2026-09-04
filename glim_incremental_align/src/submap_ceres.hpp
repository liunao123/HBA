// -----------------------------------------------------------------------------
// submap 内局部优化的 Ceres 版本
//
// 残差设计移植自 sparse-mapping/include/error_term.h (ms_mapping)。可移植的三条:
//     RelTConstraint  INS 位置软锚, **z 只给 0.75 倍权重** ("ins 的 z 轴精度较低")
//     TError          帧间相对位姿 (测量来自预先算好的 GICP/VGICP 结果)
//     AttitudePrior   把 roll/pitch 拉向 INS, **yaw 保持自由**, 按重力方向分解
// 第四条 ReprojError 是视觉重投影, 本管线没有特征观测, 无法移植。
//
// !! 与 gtsam 路径的结构差异, 必须清楚 !!
//   gtsam 路径用 IntegratedVGICPFactor: 点云代价在优化器内部, 每次线性化都**重新找对应**。
//   Ceres 路径是位姿图: GICP 先跑出一个相对位姿, 之后测量就**冻结**了。
//   前者更强(能自己修对应关系), 后者的好处是 Huber/Cauchy 鲁棒核 + 卡方剔除更直接,
//   而且 AttitudePrior 的重力分解比"在 tangent 上取对角"更正确。
//   所以这不是"Ceres 比 gtsam 好", 是两种不同的取舍。
//
// AttitudePrior 为什么值得移植:
//   我原来用 anisoInfo 在 Pose3 tangent 上取对角 (rx, ry, rz), 这等于假设车体 x/y 轴就是
//   roll/pitch、z 轴就是 yaw —— 只在车辆完全水平时成立。ms_mapping 的做法是把旋转误差
//   投影到重力方向上: 平行分量是 yaw(不约束), 垂直分量才是 roll/pitch(约束)。
//   与 LiDAR 安装姿态无关, 更正确。
// -----------------------------------------------------------------------------
#pragma once

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <glog/logging.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

namespace ialign {

/// @brief 6 维位姿参数化: [0..2] = angle-axis, [3..5] = translation (与 ms_mapping 一致)
inline void poseToParam(const Eigen::Isometry3d& T, double* p) {
  const Eigen::Matrix3d R = T.linear();
  double rot[9];
  // Ceres 期望列主序, Eigen 默认也是列主序
  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) rot[c * 3 + r] = R(r, c);
  }
  ceres::RotationMatrixToAngleAxis(rot, p);
  p[3] = T.translation().x();
  p[4] = T.translation().y();
  p[5] = T.translation().z();
}

inline Eigen::Isometry3d paramToPose(const double* p) {
  double rot[9];
  ceres::AngleAxisToRotationMatrix(p, rot);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) T.linear()(r, c) = rot[c * 3 + r];
  }
  T.translation() << p[3], p[4], p[5];
  return T;
}

/// @brief INS 位置软锚。z 权重 0.75 —— 源自 ms_mapping 的判断"ins 的 z 轴精度较低"。
struct InsPositionPrior {
  Eigen::Vector3d t_ins;
  double inv_sigma;
  double z_scale;

  InsPositionPrior(const Eigen::Vector3d& t, double sigma, double zs = 0.75)
      : t_ins(t), inv_sigma(1.0 / std::max(1e-9, sigma)), z_scale(zs) {}

  template <typename T>
  bool operator()(const T* const p, T* r) const {
    r[0] = (p[3] - T(t_ins.x())) * T(inv_sigma);
    r[1] = (p[4] - T(t_ins.y())) * T(inv_sigma);
    r[2] = (p[5] - T(t_ins.z())) * T(inv_sigma) * T(z_scale);
    return true;
  }
};

/// @brief 帧间相对位姿残差 (测量 = 预先算好的 T_i_j)
struct RelPoseError {
  Eigen::Isometry3d T_ij;
  double inv_sigma_t;
  double inv_sigma_r;

  RelPoseError(const Eigen::Isometry3d& T, double st, double sr)
      : T_ij(T), inv_sigma_t(1.0 / std::max(1e-9, st)), inv_sigma_r(1.0 / std::max(1e-9, sr)) {}

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, T* r) const {
    T Ri[9], Rj[9];
    ceres::AngleAxisToRotationMatrix(pi, Ri);
    ceres::AngleAxisToRotationMatrix(pj, Rj);
    Eigen::Matrix<T, 3, 3> mRi, mRj;
    for (int c = 0; c < 3; c++) {
      for (int rw = 0; rw < 3; rw++) {
        mRi(rw, c) = Ri[c * 3 + rw];
        mRj(rw, c) = Rj[c * 3 + rw];
      }
    }
    Eigen::Matrix<T, 3, 1> ti(pi[3], pi[4], pi[5]), tj(pj[3], pj[4], pj[5]);

    // 预测的相对位姿
    const Eigen::Matrix<T, 3, 3> Rij_pred = mRi.transpose() * mRj;
    const Eigen::Matrix<T, 3, 1> tij_pred = mRi.transpose() * (tj - ti);

    const Eigen::Matrix<T, 3, 1> dt = tij_pred - T_ij.translation().cast<T>();
    r[0] = dt(0) * T(inv_sigma_t);
    r[1] = dt(1) * T(inv_sigma_t);
    r[2] = dt(2) * T(inv_sigma_t);

    // 旋转误差取反对称部分的 vee (小角度下 ≈ 旋转向量, 平滑无奇异)
    const Eigen::Matrix<T, 3, 3> Rerr = T_ij.linear().cast<T>().transpose() * Rij_pred;
    r[3] = (Rerr(2, 1) - Rerr(1, 2)) * T(0.5) * T(inv_sigma_r);
    r[4] = (Rerr(0, 2) - Rerr(2, 0)) * T(0.5) * T(inv_sigma_r);
    r[5] = (Rerr(1, 0) - Rerr(0, 1)) * T(0.5) * T(inv_sigma_r);
    return true;
  }
};

/// @brief 姿态软先验: roll/pitch 拉向 INS, yaw 自由。按重力方向分解, 与安装姿态无关。
struct AttitudePriorG {
  Eigen::Matrix3d R_prior;
  Eigen::Vector3d g0;  // 重力"上"方向在优化所用参考系下的单位向量
  double w;

  AttitudePriorG(const Eigen::Matrix3d& R, const Eigen::Vector3d& g, double weight)
      : R_prior(R), g0(g.normalized()), w(weight) {}

  template <typename T>
  bool operator()(const T* const p, T* r) const {
    T Rm[9];
    ceres::AngleAxisToRotationMatrix(p, Rm);
    Eigen::Matrix<T, 3, 3> R_opt;
    for (int c = 0; c < 3; c++) {
      for (int rw = 0; rw < 3; rw++) R_opt(rw, c) = Rm[c * 3 + rw];
    }
    const Eigen::Matrix<T, 3, 3> R_err = R_prior.cast<T>().transpose() * R_opt;
    Eigen::Matrix<T, 3, 1> wb;
    wb(0) = (R_err(2, 1) - R_err(1, 2)) * T(0.5);
    wb(1) = (R_err(0, 2) - R_err(2, 0)) * T(0.5);
    wb(2) = (R_err(1, 0) - R_err(0, 1)) * T(0.5);
    // 转到参考系, 去掉绕 g0 的分量(yaw), 余下即 roll/pitch
    const Eigen::Matrix<T, 3, 1> w0 = R_prior.cast<T>() * wb;
    const Eigen::Matrix<T, 3, 1> g = g0.cast<T>();
    const Eigen::Matrix<T, 3, 1> wp = w0 - (w0.dot(g)) * g;
    r[0] = wp(0) * T(w);
    r[1] = wp(1) * T(w);
    r[2] = wp(2) * T(w);
    return true;
  }
};

// -----------------------------------------------------------------------------
// 视觉重投影残差 (对应 ms_mapping 的 ReprojError)
//
//   帧 i 的雷达系里一个**固定**的 3D 点 p_i, 经两个位姿变量投到帧 j 的图像,
//   与帧 j 上匹配到的像素比。残差 2 维, 只连 T_i / T_j, 不引入路标变量。
//
// 为什么要它: 沿路方向是激光的退化方向 —— 沿路平移时最近邻代价近乎平坦, GICP 在这个
// 方向没有约束力(本项目实测: 放开水平会让缝从 0.146 涨到 0.193)。而车道线/杆子/路牌的
// 像素位置对沿路平移极敏感, 正好补上这个方向。
//
// sigma_px 要按**实测重投影残差**给, 不是按"想让它多重要"给。本项目实测过一次:
// 假设 10px 时白化 rms=0.228 -> 真实离散度约 2.3px, 所以 sigma 该取 2.3 而不是 10。
// 下面 optimizeSubmapCeres 会按类别打白化 rms, 偏离 1 就照那个比例调。
// -----------------------------------------------------------------------------
struct VisReprojSpec {
  int i = -1, j = -1;                              // 帧下标: i 提供 3D 点, j 提供像素
  Eigen::Vector3d p_i = Eigen::Vector3d::Zero();   // 帧 i 雷达系下的 3D 点
  Eigen::Matrix4d T_c_l = Eigen::Matrix4d::Identity();  // 雷达 -> 相机
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double u = 0, v = 0;                             // 图像 j 上的观测像素
};

struct ReprojResidual {
  const VisReprojSpec* s = nullptr;
  double inv_sigma = 1.0;
  double z_min = 0.5;

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, T* r) const {
    // p_o = R_i * p_i + t_i   (submap origin 系)
    T p[3] = {T(s->p_i.x()), T(s->p_i.y()), T(s->p_i.z())};
    T po[3];
    ceres::AngleAxisRotatePoint(pi, p, po);
    po[0] += pi[3];
    po[1] += pi[4];
    po[2] += pi[5];
    // p_lj = R_j^T * (p_o - t_j)   (帧 j 的雷达系)
    const T d[3] = {po[0] - pj[3], po[1] - pj[4], po[2] - pj[5]};
    const T aa_inv[3] = {-pj[0], -pj[1], -pj[2]};
    T plj[3];
    ceres::AngleAxisRotatePoint(aa_inv, d, plj);
    // p_cam = T_c_l * p_lj
    T pc[3];
    for (int k = 0; k < 3; k++) {
      pc[k] = T(s->T_c_l(k, 0)) * plj[0] + T(s->T_c_l(k, 1)) * plj[1] +
              T(s->T_c_l(k, 2)) * plj[2] + T(s->T_c_l(k, 3));
    }
    // z 夹住而不是让残差失效: 建对应时已用初值筛过, z<=0 实际不会发生;
    // 真发生了给一个有界的大残差交给 Huber 压住, 而不是返回 0 让优化器以为误差降了。
    const T z = pc[2] > T(z_min) ? pc[2] : T(z_min);
    r[0] = (T(s->fx) * pc[0] / z + T(s->cx) - T(s->u)) * T(inv_sigma);
    r[1] = (T(s->fy) * pc[1] / z + T(s->cy) - T(s->v)) * T(inv_sigma);
    return true;
  }
};

// =============================================================================
// 把**雷达外参 T_v_l 也作为参数**一起优化的版本
//
// 为什么必须换模型结构: 原来的 BA 直接优化 T_w_l (雷达在世界系的位姿), 这时外参是
// **不可观测**的 —— 外参绕 z 转 δ 和"每帧位姿各自转 δ"是同一件事, 没有信息能区分。
// 改成优化 T_w_v (车体位姿) + 共享的 T_v_l, 两者由 T_w_l = T_w_v * T_v_l 相连:
//   - INS 先验约束 T_w_v (INS 测的就是车体);
//   - 点云/视觉约束落在 T_w_l 上, 于是能反推出 T_v_l 该是多少。
//
// 为什么值得做 (实测动机): WG_wuling 上 yaw(T_w_l) 与实际行驶方向差 -2.19 度(直线段
// std 0.53), 而外参文件里的 yaw 只有 -0.32 度 —— 输入里有约 1.9 度的偏航不一致。
// 不修的话 BA 会把每块补丁绕各自锚定帧转 +2 度去补(实测 R^2=0.92~0.999 的刚性旋转),
// 各块因此处在不同的旋转规范系里, 锚点之间就对不上(锚间不一致 0.016~0.341m)。
// 手工给 -1.87 度能把 BA 位移降到 1/4~1/14, 但那是我从中位数估的; 放进 BA 一起解才对。
//
// 视觉项在这里有额外价值: 相机外参 T_v_c 是**独立**的, 所以
//   雷达外参偏了 -> 点云投到图像上系统性错位 -> 视觉残差会推 T_v_l;
//   INS 航向偏置 -> 相机和雷达一起转 -> 视觉残差不受影响。
// 于是"外参错"和"航向偏置"这两个原本混在一起的解释被分开了。
//
// 平移默认**不优化**: 杆臂平移在一条直路上和位置强耦合(病态), ms_mapping 也是把外参
// 平移固定的。--opt_ext full 可以放开, 但要盯住它是否跑飞。
// =============================================================================

/// @brief 帧间相对位姿残差, 外参作为第三个参数块。
///        预测: T_li_lj = Ext^-1 * (T_wvi^-1 * T_wvj) * Ext
struct RelPoseErrorExt {
  Eigen::Isometry3d T_ij;   // 测量: GICP 量出的**雷达系之间**的相对位姿
  double inv_sigma_t, inv_sigma_r;

  RelPoseErrorExt(const Eigen::Isometry3d& T, double st, double sr)
    : T_ij(T), inv_sigma_t(1.0 / std::max(1e-9, st)), inv_sigma_r(1.0 / std::max(1e-9, sr)) {}

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, const T* const pe, T* r) const {
    const auto mat = [](const T* p, Eigen::Matrix<T, 3, 3>& R, Eigen::Matrix<T, 3, 1>& t) {
      T rot[9];
      ceres::AngleAxisToRotationMatrix(p, rot);
      for (int c = 0; c < 3; c++) {
        for (int rw = 0; rw < 3; rw++) R(rw, c) = rot[c * 3 + rw];
      }
      t << p[3], p[4], p[5];
    };
    Eigen::Matrix<T, 3, 3> Ri, Rj, Re;
    Eigen::Matrix<T, 3, 1> ti, tj, te;
    mat(pi, Ri, ti);
    mat(pj, Rj, tj);
    mat(pe, Re, te);

    // 车体系相对位姿
    const Eigen::Matrix<T, 3, 3> Rvij = Ri.transpose() * Rj;
    const Eigen::Matrix<T, 3, 1> tvij = Ri.transpose() * (tj - ti);
    // 换算到雷达系: T_li_lj = Ext^-1 * T_vi_vj * Ext
    const Eigen::Matrix<T, 3, 3> Rlij = Re.transpose() * Rvij * Re;
    const Eigen::Matrix<T, 3, 1> tlij = Re.transpose() * (Rvij * te + tvij - te);

    const Eigen::Matrix<T, 3, 1> dt = tlij - T_ij.translation().cast<T>();
    r[0] = dt(0) * T(inv_sigma_t);
    r[1] = dt(1) * T(inv_sigma_t);
    r[2] = dt(2) * T(inv_sigma_t);
    const Eigen::Matrix<T, 3, 3> Rerr = T_ij.linear().cast<T>().transpose() * Rlij;
    r[3] = (Rerr(2, 1) - Rerr(1, 2)) * T(0.5) * T(inv_sigma_r);
    r[4] = (Rerr(0, 2) - Rerr(2, 0)) * T(0.5) * T(inv_sigma_r);
    r[5] = (Rerr(1, 0) - Rerr(0, 1)) * T(0.5) * T(inv_sigma_r);
    return true;
  }
};

/// @brief 视觉重投影, 外参作为第三个参数块。T_c_l 每次迭代现算 = T_v_c^-1 * T_v_l。
struct VisReprojSpecExt {
  int i = -1, j = -1;
  Eigen::Vector3d p_i = Eigen::Vector3d::Zero();   // 帧 i **雷达系**下的 3D 点
  Eigen::Matrix4d T_c_v = Eigen::Matrix4d::Identity();  // 车体 -> 相机 (= T_v_c^-1, 常量)
  double fx = 0, fy = 0, cx = 0, cy = 0, u = 0, v = 0;
};

struct ReprojResidualExt {
  const VisReprojSpecExt* s = nullptr;
  double inv_sigma = 1.0;
  double z_min = 0.5;

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, const T* const pe, T* r) const {
    const auto mat = [](const T* p, Eigen::Matrix<T, 3, 3>& R, Eigen::Matrix<T, 3, 1>& t) {
      T rot[9];
      ceres::AngleAxisToRotationMatrix(p, rot);
      for (int c = 0; c < 3; c++) {
        for (int rw = 0; rw < 3; rw++) R(rw, c) = rot[c * 3 + rw];
      }
      t << p[3], p[4], p[5];
    };
    Eigen::Matrix<T, 3, 3> Ri, Rj, Re;
    Eigen::Matrix<T, 3, 1> ti, tj, te;
    mat(pi, Ri, ti);
    mat(pj, Rj, tj);
    mat(pe, Re, te);

    // p_i (雷达系 i) -> 车体系 i -> 世界 -> 车体系 j -> 相机 j
    const Eigen::Matrix<T, 3, 1> p_li = s->p_i.cast<T>();
    const Eigen::Matrix<T, 3, 1> p_vi = Re * p_li + te;
    const Eigen::Matrix<T, 3, 1> p_w = Ri * p_vi + ti;
    const Eigen::Matrix<T, 3, 1> p_vj = Rj.transpose() * (p_w - tj);
    Eigen::Matrix<T, 3, 1> pc;
    for (int k = 0; k < 3; k++) {
      pc(k) = T(s->T_c_v(k, 0)) * p_vj(0) + T(s->T_c_v(k, 1)) * p_vj(1) +
              T(s->T_c_v(k, 2)) * p_vj(2) + T(s->T_c_v(k, 3));
    }
    const T z = pc(2) > T(z_min) ? pc(2) : T(z_min);
    r[0] = (T(s->fx) * pc(0) / z + T(s->cx) - T(s->u)) * T(inv_sigma);
    r[1] = (T(s->fy) * pc(1) / z + T(s->cy) - T(s->v)) * T(inv_sigma);
    return true;
  }
};

struct SubmapCeresOpts {
  double sigma_ins_xy = 0.30;   // INS 位置锚的 sigma (m)
  double ins_z_scale = 0.9;    // z 的权重系数 (源自 ms_mapping: INS 的 z 更差)
  double sigma_rel_t = 0.10;    // 帧间相对位姿的平移 sigma (m)
  double sigma_rel_r = 0.01;    // 帧间相对位姿的旋转 sigma (rad)
  double attitude_w = 50.0;     // 姿态先验权重
  double huber = 1.0;           // Huber 核宽度
  int iters = 50;
  double chi2_reject = 0.0;     // >0 时按卡方阈值剔除异常残差块后再解一轮
  double sigma_px = 2.5;        // 视觉重投影 sigma (px)。按实测离散度给, 见 ReprojResidual 注释
  bool report = true;           // 按类别打白化残差统计
  // 外参优化: 0=不动 1=只优化旋转 2=旋转+平移。
  // 默认只优化旋转 —— 杆臂平移在直路上和位置强耦合(病态), ms_mapping 也是固定的。
  int opt_ext = 1;
};

struct SubmapCeresStats {
  int n_frames = 0;
  int n_rel = 0;
  int n_vis = 0;
  int n_removed = 0;
  double cost_before = 0.0;
  double cost_after = 0.0;
  bool ok = false;
  // 按类别的白化 rms (优化后)。**偏离 1 就说明那一类的 sigma 假设不对, 该乘上这个数** ——
  // 这样各约束的权重来自各自的实测离散度, 而不是"我希望它多重要"。
  double rms_ins = -1, rms_rel = -1, rms_att = -1, rms_vis = -1;
  double rms_rel_t = -1, rms_rel_r = -1;   // 帧间残差的平移/旋转分量分开
  // 帧间约束按 rel_group 分成两组时, 各组单独的白化 rms (校准时要分开看:
  // 同 session 与跨 session 的真实离散度差 2~3 倍, 混在一起算等于两边都校不准)
  // 平移/旋转必须分开: 一条 rel 残差里 r[0..2] 是平移、r[3..5] 是旋转, 混在一起的
  // 6 维 rms 无法告诉你该调 sigma_rel_t 还是 sigma_rel_r。
  // 组号含义 (由调用方给, 见 local_align.cpp 里 grp 的构造):
  //   0 = 同 session 窗口内帧间   1 = 跨 session scan2submap
  //   2 = 同向回环               3 = **反向**回环
  // 回环必须单列: 它和窗口内帧间是两类完全不同的测量 (窗口内是相邻几米、同向、重叠高;
  // 回环跨几百米弧长, 反向的还只能看到物体相反那一面), 真实离散度差好几倍。
  // 混在一组里算 rms, 3.5 万条窗口边会把两百条回环边完全淹掉 —— 于是回环的 sigma
  // 永远校不出来, 只能沿用窗口边的 0.035, 而那个值对它是错的。
  double rms_g0_t = -1, rms_g0_r = -1, rms_g1_t = -1, rms_g1_r = -1;
  double rms_g2_t = -1, rms_g2_r = -1, rms_g3_t = -1, rms_g3_r = -1;
  int n_rel_g0 = 0, n_rel_g1 = 0, n_rel_g2 = 0, n_rel_g3 = 0;
  int n_fixed = 0;               // 被固定住的位姿数 (增量建图里 = 前面已优化 session 的帧数)
};

// -----------------------------------------------------------------------------
/// @brief 把外参也当参数的 BA。优化 T_w_v (车体位姿) + 共享 T_v_l。
/// @param T_w_v      进/出: 车体位姿 (初值 <- INS)
/// @param T_v_l      进/出: 雷达外参 (初值 <- yaml)
/// @param rel        测量: **雷达系之间**的相对位姿 (GICP 量出来的)
/// @param center     哪一帧当锚 (它的车体位姿被固定, 定规范自由度)
inline SubmapCeresStats optimizeSubmapCeresExt(
  std::vector<Eigen::Isometry3d>& T_w_v,
  Eigen::Isometry3d& T_v_l,
  const std::vector<Eigen::Isometry3d>& T_w_v_init,
  const Eigen::Isometry3d& T_v_l_init,
  const std::vector<std::tuple<int, int, Eigen::Isometry3d>>& rel,
  const Eigen::Vector3d& g0,
  int center,
  const SubmapCeresOpts& o,
  const std::vector<VisReprojSpecExt>& vis = {},
  /// 每条 rel 约束各自的 (sigma_t, sigma_r)。长度与 rel 一致时生效, 否则退回 o.sigma_rel_*。
  /// 有了这个就不需要"把约束重复 k 次来放大信息矩阵"那种近似了。
  const std::vector<std::array<double, 2>>& rel_sigma = {},
  /// 每条 rel 约束的组号 (0..3: 窗口内/跨session/同向回环/反向回环)。
  /// **只用于分组统计白化 rms, 不影响求解** —— 权重走 rel_sigma。
  const std::vector<int>& rel_group = {},
  /// 哪些帧的位姿**固定不动** (增量建图: 前面已优化好的 session 作为参照)。
  /// 固定帧不加 INS/姿态先验 —— 它们的位姿是给定的, 加先验只会污染残差统计。
  const std::vector<char>& fixed = {},
  /// INS/姿态先验的**测量值**。不给就用 T_w_v_init。
  /// 为什么要和初值分开: 增量建图里, 已优化过的 session 再次参与优化时, 初值是它上一轮
  /// 的优化结果, 但它的 INS 先验测的仍然是**当初 INS 给的位置**。混用的话先验会变成
  /// "把位姿钉在自己上一轮的结果上" —— 一个自证的假约束, 越迭代越僵。
  const std::vector<Eigen::Isometry3d>* prior_pose = nullptr) {
  SubmapCeresStats st;
  const int n = static_cast<int>(T_w_v_init.size());
  if (n < 2 || rel.empty()) return st;
  st.n_frames = n;

  std::vector<std::array<double, 6>> par(n);
  for (int i = 0; i < n; i++) poseToParam(T_w_v_init[i], par[i].data());
  std::array<double, 6> pe{};
  poseToParam(T_v_l_init, pe.data());

  ceres::Problem::Options po;
  po.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(po);
  ceres::HuberLoss huber(o.huber);
  ceres::CauchyLoss cauchy(1.0);
  ceres::LossFunction* loss = o.huber > 0 ? &huber : nullptr;

  const bool has_fixed = fixed.size() == static_cast<std::size_t>(n);
  for (int i = 0; i < n; i++) problem.AddParameterBlock(par[i].data(), 6);
  problem.AddParameterBlock(pe.data(), 6);
  if (center >= 0 && center < n) problem.SetParameterBlockConstant(par[center].data());
  if (has_fixed) {
    for (int i = 0; i < n; i++) {
      if (!fixed[i]) continue;
      problem.SetParameterBlockConstant(par[i].data());
      st.n_fixed++;
    }
  }
  if (o.opt_ext == 0) {
    problem.SetParameterBlockConstant(pe.data());
  } else if (o.opt_ext == 1) {
    // 只放开旋转: 用 SubsetManifold 固定后三维(平移)
    // Ceres 2.0: 用 SetParameterization + SubsetParameterization
    // (SetManifold/SubsetManifold 是 2.1+ 才有的, 这里装的是 2.0)
    problem.SetParameterization(pe.data(),
                                new ceres::SubsetParameterization(6, std::vector<int>{3, 4, 5}));
  }

  std::vector<ceres::ResidualBlockId> ins_ids, att_ids, rel_ids, vis_ids;
  for (int i = 0; i < n; i++) {
    if (i == center) continue;
    if (has_fixed && fixed[i]) continue;   // 固定帧不加先验
    const Eigen::Isometry3d& Tp =
      (prior_pose && prior_pose->size() == T_w_v_init.size()) ? (*prior_pose)[i] : T_w_v_init[i];
    ins_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<InsPositionPrior, 3, 6>(
        new InsPositionPrior(Tp.translation(), o.sigma_ins_xy, o.ins_z_scale)),
      nullptr, par[i].data()));
    att_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<AttitudePriorG, 3, 6>(
        new AttitudePriorG(Tp.linear(), g0, o.attitude_w)),
      nullptr, par[i].data()));
  }
  const bool per_sigma = rel_sigma.size() == rel.size();
  const bool per_group = rel_group.size() == rel.size();
  std::vector<ceres::ResidualBlockId> rel_g[4];
  for (std::size_t c = 0; c < rel.size(); c++) {
    const auto& [i, j, T] = rel[c];
    if (i < 0 || j < 0 || i >= n || j >= n) continue;
    const double st_ = per_sigma ? rel_sigma[c][0] : o.sigma_rel_t;
    const double sr_ = per_sigma ? rel_sigma[c][1] : o.sigma_rel_r;
    const auto id = problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RelPoseErrorExt, 6, 6, 6, 6>(
        new RelPoseErrorExt(T, st_, sr_)),
      loss, par[i].data(), par[j].data(), pe.data());
    rel_ids.push_back(id);
    if (per_group) rel_g[std::min(3, std::max(0, rel_group[c]))].push_back(id);
  }
  st.n_rel = static_cast<int>(rel_ids.size());
  if (rel_ids.empty()) return st;
  for (const auto& v : vis) {
    if (v.i < 0 || v.j < 0 || v.i >= n || v.j >= n || v.i == v.j) continue;
    ReprojResidualExt rr;
    rr.s = &v;
    rr.inv_sigma = 1.0 / std::max(1e-6, o.sigma_px);
    vis_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<ReprojResidualExt, 2, 6, 6, 6>(new ReprojResidualExt(rr)),
      &cauchy, par[v.i].data(), par[v.j].data(), pe.data()));
  }
  st.n_vis = static_cast<int>(vis_ids.size());

  // glog 的 VLOG 与 Ceres 的 logging_type 是两套 —— 只设后者的话线性求解器的计时表
  // 还是会刷屏(实测把有用的日志完全淹没)。ms_mapping 也是这么关的。
  // 不要调 InitGoogleLogging —— 它被调用第二次会直接 abort, 而这两处都可能被多次进入。
  // 只设 FLAGS 就够了。
  FLAGS_minloglevel = google::ERROR;
  FLAGS_stderrthreshold = google::ERROR;
  ceres::Solver::Options so;
  so.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  so.max_num_iterations = o.iters;
  so.num_threads = 1;
  so.logging_type = ceres::SILENT;
  ceres::Solver::Summary sum;
  ceres::Solve(so, &problem, &sum);
  st.cost_before = sum.initial_cost;
  st.cost_after = sum.final_cost;
  st.ok = true;

  const auto wr = [&](const std::vector<ceres::ResidualBlockId>& ids, int dim) {
    if (ids.empty()) return -1.0;
    ceres::Problem::EvaluateOptions eo;
    eo.residual_blocks = ids;
    eo.apply_loss_function = false;
    double c = 0;
    std::vector<double> v;
    if (!problem.Evaluate(eo, &c, &v, nullptr, nullptr)) return -1.0;
    double sq = 0;
    for (const double x : v) sq += x * x;
    return std::sqrt(sq / std::max<std::size_t>(1, ids.size() * dim));
  };
  st.rms_ins = wr(ins_ids, 3);
  st.rms_att = wr(att_ids, 3);
  st.rms_rel = wr(rel_ids, 6);
  st.rms_vis = wr(vis_ids, 2);
  // 分组 + 平移/旋转分离的白化 rms
  const auto wr_tr = [&](const std::vector<ceres::ResidualBlockId>& ids, double& rt, double& rr) {
    rt = rr = -1.0;
    if (ids.empty()) return;
    ceres::Problem::EvaluateOptions eo;
    eo.residual_blocks = ids;
    eo.apply_loss_function = false;
    double c = 0;
    std::vector<double> v;
    if (!problem.Evaluate(eo, &c, &v, nullptr, nullptr)) return;
    double st_ = 0, sr_ = 0;
    for (std::size_t b = 0; b + 5 < v.size(); b += 6) {
      for (int k = 0; k < 3; k++) st_ += v[b + k] * v[b + k];
      for (int k = 3; k < 6; k++) sr_ += v[b + k] * v[b + k];
    }
    const double m = static_cast<double>(ids.size()) * 3.0;
    rt = std::sqrt(st_ / m);
    rr = std::sqrt(sr_ / m);
  };
  wr_tr(rel_g[0], st.rms_g0_t, st.rms_g0_r);
  wr_tr(rel_g[1], st.rms_g1_t, st.rms_g1_r);
  wr_tr(rel_g[2], st.rms_g2_t, st.rms_g2_r);
  wr_tr(rel_g[3], st.rms_g3_t, st.rms_g3_r);
  st.n_rel_g0 = static_cast<int>(rel_g[0].size());
  st.n_rel_g1 = static_cast<int>(rel_g[1].size());
  st.n_rel_g2 = static_cast<int>(rel_g[2].size());
  st.n_rel_g3 = static_cast<int>(rel_g[3].size());
  if (o.report) {
    printf("    白化残差: INS %.3f | 姿态 %.3f | 帧间 %.3f | 视觉 %.3f"
           "   (rms 偏离 1 = 该类 sigma 该乘上它)\n",
           st.rms_ins, st.rms_att, st.rms_rel, st.rms_vis);
    if (st.n_rel_g0)
      printf("      同session 帧间: 平移 %.3f 旋转 %.3f (%d 条)%s\n", st.rms_g0_t, st.rms_g0_r,
             st.n_rel_g0,
             st.n_rel_g1 ? "" : "   [没有跨 session 约束]");
    if (st.n_rel_g1)
      printf("      跨session 帧间: 平移 %.3f 旋转 %.3f (%d 条)\n", st.rms_g1_t, st.rms_g1_r,
             st.n_rel_g1);
    if (st.n_rel_g2)
      printf("      同向回环:       平移 %.3f 旋转 %.3f (%d 条)\n", st.rms_g2_t, st.rms_g2_r,
             st.n_rel_g2);
    if (st.n_rel_g3)
      printf("      **反向回环**:   平移 %.3f 旋转 %.3f (%d 条)\n", st.rms_g3_t, st.rms_g3_r,
             st.n_rel_g3);
  }

  T_w_v.resize(n);
  for (int i = 0; i < n; i++) T_w_v[i] = paramToPose(par[i].data());
  T_v_l = paramToPose(pe.data());
  return st;
}

/// @brief 在 submap 局部系里优化各帧位姿。
/// @param T_ol_init  初值 (帧相对 submap origin)
/// @param rel        预先算好的帧间相对位姿约束 (i, j, T_i_j)
/// @param g0         重力上方向在 submap origin 系下的单位向量
/// @param center     哪一帧当锚 (它被固定)
inline SubmapCeresStats optimizeSubmapCeres(
  std::vector<Eigen::Isometry3d>& T_ol,
  const std::vector<Eigen::Isometry3d>& T_ol_init,
  const std::vector<std::tuple<int, int, Eigen::Isometry3d>>& rel,
  const Eigen::Vector3d& g0,
  int center,
  const SubmapCeresOpts& o,
  const std::vector<VisReprojSpec>& vis = {}) {
  SubmapCeresStats st;
  const int n = static_cast<int>(T_ol_init.size());
  if (n < 2 || rel.empty()) return st;
  st.n_frames = n;

  std::vector<std::array<double, 6>> par(n);
  for (int i = 0; i < n; i++) poseToParam(T_ol_init[i], par[i].data());

  ceres::Problem::Options po;
  po.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(po);
  ceres::HuberLoss huber(o.huber);
  ceres::LossFunction* loss = o.huber > 0 ? &huber : nullptr;

  for (int i = 0; i < n; i++) problem.AddParameterBlock(par[i].data(), 6);
  if (center >= 0 && center < n) problem.SetParameterBlockConstant(par[center].data());

  // INS 位置软锚 + 姿态先验(roll/pitch, yaw 自由)
  std::vector<ceres::ResidualBlockId> ins_ids, att_ids;
  for (int i = 0; i < n; i++) {
    if (i == center) continue;
    ins_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<InsPositionPrior, 3, 6>(
        new InsPositionPrior(T_ol_init[i].translation(), o.sigma_ins_xy, o.ins_z_scale)),
      nullptr, par[i].data()));
    att_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<AttitudePriorG, 3, 6>(
        new AttitudePriorG(T_ol_init[i].linear(), g0, o.attitude_w)),
      nullptr, par[i].data()));
  }

  std::vector<ceres::ResidualBlockId> rel_ids;
  for (const auto& [i, j, T] : rel) {
    if (i < 0 || j < 0 || i >= n || j >= n) continue;
    rel_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RelPoseError, 6, 6, 6>(
        new RelPoseError(T, o.sigma_rel_t, o.sigma_rel_r)),
      loss, par[i].data(), par[j].data()));
  }
  st.n_rel = static_cast<int>(rel_ids.size());
  if (rel_ids.empty()) return st;

  // ---- 视觉重投影 ----
  // spec 存在调用方的 vector 里, 这里只持指针, 所以 vis 必须活到 Solve 之后 —— 它是
  // 函数参数(引用), 调用方的对象在整个调用期间有效, 满足。
  std::vector<ceres::ResidualBlockId> vis_ids;
  ceres::CauchyLoss cauchy(1.0);   // 视觉用 Cauchy 而非 Huber, 与 ms_mapping 一致
  for (const auto& v : vis) {
    if (v.i < 0 || v.j < 0 || v.i >= n || v.j >= n || v.i == v.j) continue;
    ReprojResidual rr;
    rr.s = &v;
    rr.inv_sigma = 1.0 / std::max(1e-6, o.sigma_px);
    vis_ids.push_back(problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<ReprojResidual, 2, 6, 6>(new ReprojResidual(rr)),
      &cauchy, par[v.i].data(), par[v.j].data()));
  }
  st.n_vis = static_cast<int>(vis_ids.size());

  // glog 的 VLOG 与 Ceres 的 logging_type 是两套 —— 只设后者的话线性求解器的计时表
  // 还是会刷屏(实测把有用的日志完全淹没)。ms_mapping 也是这么关的。
  // 不要调 InitGoogleLogging —— 它被调用第二次会直接 abort, 而这两处都可能被多次进入。
  // 只设 FLAGS 就够了。
  FLAGS_minloglevel = google::ERROR;
  FLAGS_stderrthreshold = google::ERROR;
  ceres::Solver::Options so;
  so.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  so.max_num_iterations = o.iters;
  so.num_threads = 1;  // 外层已按 submap 并行
  so.logging_type = ceres::SILENT;

  ceres::Solver::Summary sum;
  ceres::Solve(so, &problem, &sum);
  st.cost_before = sum.initial_cost;
  st.cost_after = sum.final_cost;

  // ---- 按类别的白化残差统计 ----
  // 每类残差按**自己假设的 sigma** 白化后 rms 应当 ≈ 1: 偏离 1 就说明那一类的 sigma
  // 假设不对, 该乘上这个 rms。这样权重来自各自的实测离散度, 而不是"我希望它多重要"。
  // **apply_loss_function=false** —— 要的是原始离散度, 不是被鲁棒核压过的。
  const auto whitenedRms = [&](const std::vector<ceres::ResidualBlockId>& ids, int dim) {
    if (ids.empty()) return -1.0;
    ceres::Problem::EvaluateOptions eo;
    eo.residual_blocks = ids;
    eo.apply_loss_function = false;
    double cost = 0;
    std::vector<double> rr;
    if (!problem.Evaluate(eo, &cost, &rr, nullptr, nullptr)) return -1.0;
    double sq = 0;
    for (const double v : rr) sq += v * v;
    return std::sqrt(sq / std::max<std::size_t>(1, ids.size() * dim));
  };
  // 6 维的帧间残差必须**分开**看: r[0..2] 是平移(按 sigma_rel_t 白化),
  // r[3..5] 是旋转(按 sigma_rel_r)。混在一起算出的 rms 既不能当平移 sigma 也不能当旋转
  // sigma —— 之前就是这么把 0.336 当成"平移实测 0.034m"报出去的, 那一步是不严谨的。
  const auto relSplitRms = [&](double& rt, double& rr2) {
    rt = rr2 = -1.0;
    if (rel_ids.empty()) return;
    ceres::Problem::EvaluateOptions eo;
    eo.residual_blocks = rel_ids;
    eo.apply_loss_function = false;
    double cost = 0;
    std::vector<double> v;
    if (!problem.Evaluate(eo, &cost, &v, nullptr, nullptr)) return;
    if (v.size() != rel_ids.size() * 6) return;
    double st2 = 0, sr2 = 0;
    for (std::size_t b = 0; b < rel_ids.size(); b++) {
      for (int k = 0; k < 3; k++) st2 += v[b * 6 + k] * v[b * 6 + k];
      for (int k = 3; k < 6; k++) sr2 += v[b * 6 + k] * v[b * 6 + k];
    }
    rt = std::sqrt(st2 / (rel_ids.size() * 3));
    rr2 = std::sqrt(sr2 / (rel_ids.size() * 3));
  };
  st.rms_ins = whitenedRms(ins_ids, 3);
  st.rms_att = whitenedRms(att_ids, 3);
  st.rms_rel = whitenedRms(rel_ids, 6);
  st.rms_vis = whitenedRms(vis_ids, 2);
  relSplitRms(st.rms_rel_t, st.rms_rel_r);
  if (o.report) {
    printf("    Ceres 白化残差 (rms 偏离 1 = 那一类的 sigma 假设不对, 该乘上它):\n"
           "      INS位置  块=%-5zu rms=%-6.3f 实测sigma≈%.4g m   (假设 %.3g)\n"
           "      姿态     块=%-5zu rms=%-6.3f\n"
           "      帧间GICP 块=%-5zu rms=%-6.3f | 平移 rms=%.3f 实测sigma≈%.4g m (假设 %.3g)\n"
           "                            | 旋转 rms=%.3f 实测sigma≈%.4g rad (假设 %.3g)\n"
           "      视觉     块=%-5zu rms=%-6.3f 实测sigma≈%.4g px  (假设 %.3g)\n",
           ins_ids.size(), st.rms_ins, st.rms_ins > 0 ? st.rms_ins * o.sigma_ins_xy : -1.0,
           o.sigma_ins_xy, att_ids.size(), st.rms_att, rel_ids.size(), st.rms_rel,
           st.rms_rel_t, st.rms_rel_t > 0 ? st.rms_rel_t * o.sigma_rel_t : -1.0, o.sigma_rel_t,
           st.rms_rel_r, st.rms_rel_r > 0 ? st.rms_rel_r * o.sigma_rel_r : -1.0, o.sigma_rel_r,
           vis_ids.size(), st.rms_vis, st.rms_vis > 0 ? st.rms_vis * o.sigma_px : -1.0, o.sigma_px);
    printf("      校准规则: sigma_new = sigma_old x rms (以权重给的项则 w_new = w_old / rms)。\n"
           "        rms<1 = 实际残差比假设的噪声**小**, 即权重给**轻**了 —— 方向别搞反。\n");
  }

  // 卡方剔除: 把马氏距离过大的相对位姿块去掉再解一轮 (对应 ms_mapping 的 remove_outliers)
  if (o.chi2_reject > 0.0 && rel_ids.size() > 4) {
    std::vector<ceres::ResidualBlockId> keep;
    for (const auto id : rel_ids) {
      double cost = 0.0;
      double res[6] = {0, 0, 0, 0, 0, 0};
      problem.EvaluateResidualBlock(id, false, &cost, res, nullptr);
      double d2 = 0.0;
      for (int k = 0; k < 6; k++) d2 += res[k] * res[k];
      if (d2 > o.chi2_reject) {
        problem.RemoveResidualBlock(id);
        st.n_removed++;
      } else {
        keep.push_back(id);
      }
    }
    if (st.n_removed > 0 && keep.size() >= 3) {
      ceres::Solve(so, &problem, &sum);
      st.cost_after = sum.final_cost;
    }
  }

  T_ol.resize(n);
  for (int i = 0; i < n; i++) T_ol[i] = paramToPose(par[i].data());
  st.ok = true;
  return st;
}

}  // namespace ialign

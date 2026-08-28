#pragma once
// -----------------------------------------------------------------------------
// 从 glim 原样搬过来的唯一一份代码 (glim/include/glim/common/cloud_covariance_estimation.hpp
// + src/glim/common/cloud_covariance_estimation.cpp), 只改了三处:
//   namespace glim -> ialign, include 改相对路径, spdlog::critical -> fprintf(stderr)。
//
// 为什么不用 gtsam_points 自带的 covariance_estimation: 这里的正则化方式
// (RegularizationMethod::PLANE, 把最小特征值抬到 1e-3) 决定了平面上的点得到"扁"协方差,
// GICP 的 plane-to-plane 行为就来自它。换实现会改变**所有**配准数值, 之前所有实测结论
// (墙面厚度 / 跨session nn / 各组白化 rms) 就全部不可比了。所以逐行搬过来。
// -----------------------------------------------------------------------------

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ialign {

/**
 * @brief Covariance regularization method
 */
enum class RegularizationMethod { NONE, PLANE, NORMALIZED_MIN_EIG, FROBENIUS };

/**
 * @brief Point covariance estimation
 */
class CloudCovarianceEstimation {
public:
  CloudCovarianceEstimation(const int num_threads = 1);
  ~CloudCovarianceEstimation();

  /**
   * @brief Estimate point normals and covariances
   * @param points    Input points
   * @param neighbors Neighbor indices (must be N * k, where N is the number of points)
   * @param normals   [output] Estimated normals
   * @param covs      [output] Estimated covariances
   */
  void estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, std::vector<Eigen::Vector4d>& normals, std::vector<Eigen::Matrix4d>& covs) const;

  /**
   * @brief Estimate point normals and covariances
   * @param points      Input points
   * @param neighbors   Neighbor indices (must be N * m, where N is the number of points)
   * @param k_neighbors Number of neighbors used for estimation (must be <= m)
   * @param normals     [output] Estimated normals
   * @param covs        [output] Estimated covariances
   */
  void estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<int>& neighbors,
    const int k_neighbors,
    std::vector<Eigen::Vector4d>& normals,
    std::vector<Eigen::Matrix4d>& covs) const;

  /// Estimate point covariances
  std::vector<Eigen::Matrix4d> estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, const int k_neighbors) const;

  /// Estimate point covariances
  std::vector<Eigen::Matrix4d> estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors) const;

  /**
   * @brief Regularize a covariance matrix
   * @param cov          Input covariance matrix
   * @param eigenvalues  [output] Eigenvalues of the covariance matrix
   * @param eigenvectors [output] Eigenvectors of the covariance matrix
   * @return Regularized covariance matrix
   */
  Eigen::Matrix4d regularize(const Eigen::Matrix4d& cov, Eigen::Vector3d* eigenvalues = nullptr, Eigen::Matrix3d* eigenvectors = nullptr) const;

private:
  const RegularizationMethod regularization_method;
  const int num_threads;
};

}  // namespace ialign

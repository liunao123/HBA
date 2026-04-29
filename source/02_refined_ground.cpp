/**
 * @file 02_refined_ground.cpp
 * @brief Post-optimization ground-based per-pose Z refinement.
 */

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>
#include <patchwork/patchworkpp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include "common.hpp"

using namespace gtsam;

typedef pcl::PointXYZI pointtype;
typedef pcl::PointCloud<pointtype>::Ptr cloud_ptr;

namespace {

bool g_enable_global_ground_z_refine = true;
float g_global_ground_search_radius_m = 50.0f;
int g_global_ground_min_neighbor_frames = 3;
int g_global_ground_min_local_map_points = 4000;
int g_global_ground_min_map_ground_points = 800;
int g_global_ground_min_frame_ground_points = 300;
int g_global_ground_min_residual_samples = 80;
float g_global_ground_max_pose_z_adjust_m = 0.10f;
int g_ground_plane_max_points = 6000;
float g_local_min_distance = 2.50f;
float g_local_max_distance = 50.0f;
float g_local_min_z = -2.30f;
float g_local_max_z = 20.0f;
float g_local_voxel_size = 0.15f;

std::unique_ptr<patchwork::PatchWorkpp> g_patchwork_global_map_instance;
std::unique_ptr<patchwork::PatchWorkpp> g_patchwork_global_frame_instance;

struct KeyedTumPose {
	Key key;
	TumPose pose;
};

struct GroundPlaneEstimate {
	bool valid = false;
	Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
	Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
};

Eigen::MatrixXf cloudToPatchworkMatrix(const cloud_ptr& cloud) {
	if (!cloud || cloud->empty()) {
		return Eigen::MatrixXf(0, 4);
	}

	Eigen::MatrixXf matrix(static_cast<int>(cloud->size()), 4);
	for (size_t index = 0; index < cloud->size(); ++index) {
		const auto& point = cloud->points[index];
		matrix(static_cast<int>(index), 0) = point.x;
		matrix(static_cast<int>(index), 1) = point.y;
		matrix(static_cast<int>(index), 2) = point.z;
		matrix(static_cast<int>(index), 3) = point.intensity;
	}
	return matrix;
}

Eigen::MatrixX3f sampleGroundPoints(const Eigen::MatrixX3f& points, int max_points) {
    return points;

	// if (points.rows() <= 0 || max_points <= 0 || points.rows() <= max_points) {
	// 	return points;
	// }

	// Eigen::MatrixX3f sampled(max_points, 3);
	// const int total_rows = static_cast<int>(points.rows());
	// const int max_idx = total_rows - 1;
	// const float step = static_cast<float>(max_idx) / static_cast<float>(max_points - 1);
	// for (int i = 0; i < max_points; ++i) {
	// 	const int idx = std::min(max_idx, static_cast<int>(std::round(i * step)));
	// 	sampled.row(i) = points.row(idx);
	// }
	// return sampled;
}

GroundPlaneEstimate fitGroundPlaneSafe(const Eigen::MatrixX3f& ground_points) {
	GroundPlaneEstimate estimate;
	if (ground_points.rows() < 20) {
		return estimate;
	}

	std::vector<Eigen::Vector3f> valid_points;
	valid_points.reserve(static_cast<size_t>(ground_points.rows()));
	for (int i = 0; i < ground_points.rows(); ++i) {
		const Eigen::Vector3f p = ground_points.row(i).transpose();
		if (std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z())) {
			valid_points.push_back(p);
		}
	}

	if (valid_points.size() < 20) {
		return estimate;
	}

	Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
	for (const auto& p : valid_points) {
		centroid += p;
	}
	centroid /= static_cast<float>(valid_points.size());

	Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
	for (const auto& p : valid_points) {
		const Eigen::Vector3f d = p - centroid;
		covariance += d * d.transpose();
	}
	covariance /= std::max(1.0f, static_cast<float>(valid_points.size() - 1));

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
	if (solver.info() != Eigen::Success) {
		return estimate;
	}

	estimate.centroid = centroid;
	estimate.normal = solver.eigenvectors().col(0).normalized();
	if (estimate.normal.z() < 0.0f) {
		estimate.normal = -estimate.normal;
	}
	estimate.valid = true;
	return estimate;
}

float medianOf(std::vector<float>& values) {
	if (values.empty()) {
		return std::numeric_limits<float>::quiet_NaN();
	}
	const size_t mid = values.size() / 2;
	std::nth_element(values.begin(), values.begin() + mid, values.end());
	float med = values[mid];
	if (values.size() % 2 == 0 && mid > 0) {
		auto max_it = std::max_element(values.begin(), values.begin() + mid);
		med = 0.5f * (med + *max_it);
	}
	return med;
}

void applyAdvancedVoxelFilterLocal(const pcl::PointCloud<pointtype>& input_cloud,
								   pcl::PointCloud<pointtype>& output_cloud) {
	output_cloud.clear();
	output_cloud.reserve(input_cloud.size());

	const float min_distance = g_local_min_distance;
	const float max_distance = g_local_max_distance;
	const float min_z = g_local_min_z;
	const float max_z = g_local_max_z;

	for (const auto& p : input_cloud.points) {
		if (std::abs(p.x) < min_distance && std::abs(p.y) < min_distance) {
			continue;
		}
		if (std::abs(p.x) > max_distance || std::abs(p.y) > max_distance) {
			continue;
		}
		if (p.z < min_z || p.z > max_z) {
			continue;
		}

		pcl::PointXYZI point;
		point.x = p.x;
		point.y = p.y;
		point.z = p.z;
		point.intensity = p.intensity;
		output_cloud.points.push_back(point);
	}

	pcl::VoxelGrid<pointtype> voxel_grid;
	voxel_grid.setLeafSize(g_local_voxel_size, g_local_voxel_size, g_local_voxel_size);
	voxel_grid.setInputCloud(output_cloud.makeShared());
	voxel_grid.filter(output_cloud);
}

cloud_ptr loadPCDForIdTimestampLocal(const std::string& pcd_dir, uint64_t id, double timestamp) {
	std::ostringstream ss;
	ss << pcd_dir;
	if (!pcd_dir.empty() && pcd_dir.back() != '/' && pcd_dir.back() != '\\') {
		ss << '/';
	}
	ss << id << "_" << std::fixed << std::setprecision(3) << timestamp << ".pcd";

	cloud_ptr raw_cloud(new pcl::PointCloud<pointtype>());
	if (pcl::io::loadPCDFile<pointtype>(ss.str(), *raw_cloud) == -1) {
		return cloud_ptr(new pcl::PointCloud<pointtype>());
	}

	cloud_ptr cropped(new pcl::PointCloud<pointtype>());
	const float crop_near_range = 5.0f;
	for (const auto& p : raw_cloud->points) {
		if (std::abs(p.x) < crop_near_range && std::abs(p.y) < crop_near_range) {
			continue;
		}
		if (std::abs(p.x) > 100.0f || std::abs(p.y) > 100.0f) {
			continue;
		}

		pcl::PointXYZI point;
		point.x = p.x;
		point.y = p.y;
		point.z = p.z;
		point.intensity = p.intensity;
		cropped->points.push_back(point);
	}

	cropped->width = static_cast<uint32_t>(cropped->points.size());
	cropped->height = 1;
	cropped->is_dense = raw_cloud->is_dense;
	return cropped;
}

patchwork::Params makePatchworkParams() {
	patchwork::Params params;
	params.verbose = false;
	params.min_range = 3;
	params.max_range = 50;
	return params;
}

bool extractGroundWithPatchwork(const cloud_ptr& cloud,
								patchwork::PatchWorkpp& patchwork_instance,
								Eigen::MatrixX3f& ground,
								const std::string& tag) {
	Eigen::MatrixXf patchwork_cloud = cloudToPatchworkMatrix(cloud);
	if (patchwork_cloud.rows() < 20) {
		std::cout << "[global ground z] skip: insufficient patchwork input points"
				  << " tag=" << tag
				  << " rows=" << patchwork_cloud.rows() << std::endl;
		return false;
	}

	patchwork_instance.estimateGround(patchwork_cloud);
	ground = patchwork_instance.getGround();
	return true;
}

std::vector<KeyedTumPose> collectKeyedTumPoses(const Values& values,
											   const std::map<Key, double>& timestamps) {
	std::map<Key, Pose3> sorted_poses;
	for (const auto& key_value : values) {
		if (values.exists<Pose3>(key_value.key)) {
			sorted_poses[key_value.key] = values.at<Pose3>(key_value.key);
		}
	}

	std::vector<KeyedTumPose> poses;
	poses.reserve(sorted_poses.size());
	for (const auto& pair : sorted_poses) {
		const Pose3& pose = pair.second;
		const Vector3 translation = pose.translation();
		const gtsam::Quaternion rotation = pose.rotation().toQuaternion();

		double timestamp = static_cast<double>(pair.first);
		auto ts_it = timestamps.find(pair.first);
		if (ts_it != timestamps.end()) {
			timestamp = ts_it->second;
		}

		TumPose tum_pose;
		tum_pose.timestamp = timestamp;
		tum_pose.t = Eigen::Vector3d(translation.x(), translation.y(), translation.z());
		tum_pose.q = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());
		poses.push_back({pair.first, tum_pose});
	}
	return poses;
}

void updateValuesFromKeyedTumPoses(Values& values, const std::vector<KeyedTumPose>& poses) {
	for (const auto& keyed_pose : poses) {
		const Eigen::Quaterniond q = keyed_pose.pose.q.normalized();
		const gtsam::Pose3 updated_pose(
			gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
			gtsam::Point3(keyed_pose.pose.t.x(), keyed_pose.pose.t.y(), keyed_pose.pose.t.z()));
		values.update(keyed_pose.key, updated_pose);
	}
}

cloud_ptr transformCloudToCenteredWorld(const cloud_ptr& pts_local,
										const TumPose& pose,
										const Eigen::Vector3f& center_translation) {
	cloud_ptr pts_world_centered(new pcl::PointCloud<pointtype>());
	if (!pts_local || pts_local->empty()) {
		return pts_world_centered;
	}

	Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
	pose_matrix.block<3, 3>(0, 0) = pose.q.normalized().toRotationMatrix().cast<float>();
	pose_matrix.block<3, 1>(0, 3) = pose.t.cast<float>() - center_translation;
	pcl::transformPointCloud(*pts_local, *pts_world_centered, pose_matrix);
	return pts_world_centered;
}

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

/**
 * @brief 将 Eigen::MatrixX3f 格式的点云保存为 PCD 文件
 * @param local_map_ground_sampled Eigen 点云矩阵 N×3，每行 x y z
 * @param pcd_file_path 保存的 pcd 文件路径（如 "ground.pcd"）
 * @return 保存成功返回 true，失败返回 false
 */
bool saveEigenMatrixToPCD(const Eigen::MatrixX3f& local_map_ground_sampled, const std::string& pcd_file_path)
{
    // 1. 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 2. 设置点云大小
    int num_points = local_map_ground_sampled.rows();
    cloud->resize(num_points);

    // 3. 把 Eigen 矩阵逐行复制到 PCL 点云
    for (int i = 0; i < num_points; ++i)
    {
        cloud->points[i].x = local_map_ground_sampled(i, 0);
        cloud->points[i].y = local_map_ground_sampled(i, 1);
        cloud->points[i].z = local_map_ground_sampled(i, 2);
    }

    // 4. 保存为 PCD 文件
    if (pcl::io::savePCDFileASCII(pcd_file_path, *cloud) == 0)
    {
        std::cout << "✅ 成功保存点云到: " << pcd_file_path << std::endl;
        return true;
    }
    else
    {
        std::cerr << "❌ 保存 PCD 文件失败！" << std::endl;
        return false;
    }
}


bool estimateGlobalGroundZCorrection(const cloud_ptr& local_map_cloud,
									 const cloud_ptr& current_frame_cloud,
									 float& z_correction,
									 int pose_index) {
	z_correction = 0.0f;
	if (!local_map_cloud || !current_frame_cloud ||
		local_map_cloud->size() < static_cast<size_t>(g_global_ground_min_local_map_points) ||
		current_frame_cloud->size() < static_cast<size_t>(g_global_ground_min_frame_ground_points)) {
		return false;
	}

	const patchwork::Params params = makePatchworkParams();
	if (!g_patchwork_global_map_instance) {
		g_patchwork_global_map_instance = std::make_unique<patchwork::PatchWorkpp>(params);
	}
	if (!g_patchwork_global_frame_instance) {
		g_patchwork_global_frame_instance = std::make_unique<patchwork::PatchWorkpp>(params);
	}

	Eigen::MatrixX3f local_map_ground;
	Eigen::MatrixX3f current_frame_ground;
	if (!extractGroundWithPatchwork(local_map_cloud, *g_patchwork_global_map_instance, local_map_ground,
									"global_map_pose_" + std::to_string(pose_index))) {
		return false;
	}
	if (!extractGroundWithPatchwork(current_frame_cloud, *g_patchwork_global_frame_instance, current_frame_ground,
									"frame_pose_" + std::to_string(pose_index))) {
		return false;
	}

	if (local_map_ground.rows() < g_global_ground_min_map_ground_points ||
		current_frame_ground.rows() < g_global_ground_min_frame_ground_points) {
		return false;
	}

	const Eigen::MatrixX3f local_map_ground_sampled = sampleGroundPoints(local_map_ground, g_ground_plane_max_points);
	const Eigen::MatrixX3f current_frame_ground_sampled = sampleGroundPoints(current_frame_ground, g_ground_plane_max_points);

    static int cnt = 1;
    if(cnt ++ < 5)
    {
        saveEigenMatrixToPCD(local_map_ground_sampled, "./refined_ground_debug/local_map_ground.pcd");
        saveEigenMatrixToPCD(current_frame_ground, "./refined_ground_debug/current_frame_ground.pcd");
    }

	const GroundPlaneEstimate local_map_plane = fitGroundPlaneSafe(local_map_ground_sampled);
	const GroundPlaneEstimate current_frame_plane = fitGroundPlaneSafe(current_frame_ground_sampled);
	if (!local_map_plane.valid || !current_frame_plane.valid) {
		return false;
	}

	const Eigen::Vector3f plane_normal = local_map_plane.normal;
	if (!std::isfinite(plane_normal.z()) || std::abs(plane_normal.z()) < 1e-3f) {
		return false;
	}

	const float plane_offset = -plane_normal.dot(local_map_plane.centroid);
	std::vector<float> residuals;
	residuals.reserve(static_cast<size_t>(current_frame_ground_sampled.rows()));
	for (int row = 0; row < current_frame_ground_sampled.rows(); ++row) {
		const Eigen::Vector3f point = current_frame_ground_sampled.row(row).transpose();
		if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
			continue;
		}
		residuals.push_back(plane_normal.dot(point) + plane_offset);
	}

	if (static_cast<int>(residuals.size()) < g_global_ground_min_residual_samples) {
		return false;
	}

	const float residual_median = medianOf(residuals);
	if (!std::isfinite(residual_median)) {
		return false;
	}

	z_correction = -residual_median / plane_normal.z();
	if (!std::isfinite(z_correction) || std::abs(z_correction) > g_global_ground_max_pose_z_adjust_m) {
		return false;
	}

	std::cout << std::fixed << std::setprecision(6)
			  << "[global ground z] success pose=" << pose_index
			  << " z_corr=" << z_correction
			  << " map_ground=" << local_map_ground.rows()
			  << " frame_ground=" << current_frame_ground.rows()
			  << " residual_median=" << residual_median << std::endl;
	return true;
}

} // namespace

void refinePoseZWithGlobalGround(Values& values,
								 const std::map<Key, double>& timestamps,
								 const std::string& pointclouds_path) {
	if (!g_enable_global_ground_z_refine) {
		std::cout << "[global ground z] skip: disabled" << std::endl;
		return;
	}

	std::vector<KeyedTumPose> poses = collectKeyedTumPoses(values, timestamps);
	if (poses.empty()) {
		std::cout << "[global ground z] skip: no poses available" << std::endl;
		return;
	}

	int refined_count = 0;
	int skipped_count = 0;
	for (size_t pose_index = 0; pose_index < poses.size(); ++pose_index) {
		const Eigen::Vector3f center = poses[pose_index].pose.t.cast<float>();
		cloud_ptr local_map_cloud(new pcl::PointCloud<pointtype>());
		int neighbor_frames = 0;

		for (size_t neighbor_index = 0; neighbor_index < poses.size(); ++neighbor_index) {
			if (neighbor_index == pose_index) {
				continue;
			}

			const Eigen::Vector2f delta_xy =
				(poses[neighbor_index].pose.t.head<2>() - poses[pose_index].pose.t.head<2>()).cast<float>();
			if (delta_xy.norm() > g_global_ground_search_radius_m) {
				continue;
			}

			cloud_ptr neighbor_cloud = loadPCDForIdTimestampLocal(
				pointclouds_path,
				static_cast<uint64_t>(poses[neighbor_index].key),
				poses[neighbor_index].pose.timestamp);
			if (!neighbor_cloud || neighbor_cloud->empty()) {
				continue;
			}

			cloud_ptr neighbor_filtered(new pcl::PointCloud<pointtype>());
			applyAdvancedVoxelFilterLocal(*neighbor_cloud, *neighbor_filtered);
			cloud_ptr neighbor_centered_world = transformCloudToCenteredWorld(
				neighbor_filtered,
				poses[neighbor_index].pose,
				center);
			if (!neighbor_centered_world || neighbor_centered_world->empty()) {
				continue;
			}

			*local_map_cloud += *neighbor_centered_world;
			++neighbor_frames;
		}

		if (neighbor_frames < g_global_ground_min_neighbor_frames ||
			local_map_cloud->size() < static_cast<size_t>(g_global_ground_min_local_map_points)) {
			++skipped_count;
			std::cout << "[global ground z] skip pose=" << pose_index
					  << " reason=insufficient_neighbors"
					  << " neighbors=" << neighbor_frames
					  << " local_map_points=" << local_map_cloud->size() << std::endl;
			continue;
		}

		cloud_ptr current_cloud = loadPCDForIdTimestampLocal(
			pointclouds_path,
			static_cast<uint64_t>(poses[pose_index].key),
			poses[pose_index].pose.timestamp);
		if (!current_cloud || current_cloud->empty()) {
			++skipped_count;
			std::cout << "[global ground z] skip pose=" << pose_index
					  << " reason=missing_current_cloud" << std::endl;
			continue;
		}

		cloud_ptr current_filtered(new pcl::PointCloud<pointtype>());
		applyAdvancedVoxelFilterLocal(*current_cloud, *current_filtered);
		cloud_ptr current_centered_world = transformCloudToCenteredWorld(
			current_filtered,
			poses[pose_index].pose,
			center);
		if (!current_centered_world || current_centered_world->empty()) {
			++skipped_count;
			std::cout << "[global ground z] skip pose=" << pose_index
					  << " reason=current_transform_failed" << std::endl;
			continue;
		}

		// --- Voxel filter at 0.1 m before ground extraction ---
		{
			pcl::VoxelGrid<pointtype> vg;
			vg.setLeafSize(0.1f, 0.1f, 0.1f);

			cloud_ptr local_map_voxeled(new pcl::PointCloud<pointtype>());
			vg.setInputCloud(local_map_cloud);
			vg.filter(*local_map_voxeled);
			local_map_cloud = local_map_voxeled;

			cloud_ptr current_voxeled(new pcl::PointCloud<pointtype>());
			vg.setInputCloud(current_centered_world);
			vg.filter(*current_voxeled);
			current_centered_world = current_voxeled;
		}

		// --- Debug: save local map and current frame for inspection ---
		if (pose_index % 10 == 0) {
			const std::string debug_dir = "./refined_ground_debug/";
			mkdir(debug_dir.c_str(), 0777);
			pcl::io::savePCDFileBinary(debug_dir + "pose_" + std::to_string(pose_index) + "_local_map.pcd", *local_map_cloud);
			pcl::io::savePCDFileBinary(debug_dir + "pose_" + std::to_string(pose_index) + "_current_frame.pcd", *current_centered_world);
			std::cout << "[debug] Saved local_map (" << local_map_cloud->size() << " pts) and current_frame ("
					  << current_centered_world->size() << " pts) for pose=" << pose_index
					  << " to " << debug_dir << std::endl;
		}

		float z_correction = 0.0f;
		if (!estimateGlobalGroundZCorrection(local_map_cloud, current_centered_world, z_correction,
											 static_cast<int>(pose_index))) {
			++skipped_count;
			continue;
		}

		poses[pose_index].pose.t.z() += static_cast<double>(z_correction);
		++refined_count;
	}

	updateValuesFromKeyedTumPoses(values, poses);
	std::cout << "[global ground z] summary: refined=" << refined_count
			  << " skipped=" << skipped_count
			  << " total=" << poses.size() << std::endl;
}

// Save gtsam Values (Pose3) back to TUM format.
static void saveValuesToTUM(const Values& values,
							const std::map<Key, double>& timestamps,
							const std::string& out_file) {
	std::ofstream file(out_file);
	if (!file.is_open()) {
		std::cerr << "[refined_ground] Cannot open output TUM file: " << out_file << std::endl;
		return;
	}

	std::map<Key, Pose3> sorted_poses;
	for (const auto& kv : values) {
		if (values.exists<Pose3>(kv.key)) {
			sorted_poses[kv.key] = values.at<Pose3>(kv.key);
		}
	}

	for (const auto& pair : sorted_poses) {
		const Pose3& pose = pair.second;
		const Vector3 trans = pose.translation();
		const gtsam::Quaternion q = pose.rotation().toQuaternion();

		double timestamp = static_cast<double>(pair.first);
		auto ts_it = timestamps.find(pair.first);
		if (ts_it != timestamps.end()) timestamp = ts_it->second;

		file << std::fixed << std::setprecision(3) << timestamp << " "
			 << std::setprecision(6)
			 << trans.x() << " " << trans.y() << " " << trans.z() << " "
			 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
	}
	file.close();
	std::cout << "[refined_ground] Saved " << sorted_poses.size()
			  << " refined poses to " << out_file << std::endl;
}

int main(int argc, char** argv) {
	std::string config_file = "/home/xf/Desktop/catkin_ws/src/HBA/rviz_cfg/config.yaml";
	if (argc > 1) config_file = argv[1];

	// --- Load config ---
	YAML::Node config;
	try {
		config = YAML::LoadFile(config_file);
	} catch (const std::exception& e) {
		std::cerr << "[refined_ground] Failed to load config: " << e.what() << std::endl;
		return -1;
	}

	const std::string work_dir = config["paths"]["work_dir"].as<std::string>();
	const std::string pointclouds_path = work_dir + "/pointclouds_clean";
	const std::string input_tum = work_dir + "/debug_file/opt_pose_enu.tum";
	const std::string output_tum = work_dir + "/debug_file/opt_pose_enu_ground_refined.tum";

	std::cout << "[refined_ground] config_file:       " << config_file << std::endl;
	std::cout << "[refined_ground] work_dir:          " << work_dir << std::endl;
	std::cout << "[refined_ground] pointclouds_path:  " << pointclouds_path << std::endl;
	std::cout << "[refined_ground] input_tum:         " << input_tum << std::endl;
	std::cout << "[refined_ground] output_tum:        " << output_tum << std::endl;

	// --- Optionally override voxel filter params from config ---
	if (config["voxel_filter"]) {
		g_local_min_distance = config["voxel_filter"]["min_distance"].as<float>(2.5f);
		g_local_max_distance = config["voxel_filter"]["max_distance"].as<float>(80.0f);
		g_local_min_z        = config["voxel_filter"]["min_z"].as<float>(-2.3f);
		g_local_max_z        = config["voxel_filter"]["max_z"].as<float>(20.0f);
		g_local_voxel_size   = config["voxel_filter"]["voxel_size"].as<float>(0.15f);
	}

	// Optionally override ground refinement params from config
	if (config["ground_refine"]) {
		const auto& gr = config["ground_refine"];
		g_enable_global_ground_z_refine = gr["enable"].as<bool>(true);
		g_global_ground_search_radius_m = gr["search_radius_m"].as<float>(50.0f);
		g_global_ground_min_neighbor_frames = gr["min_neighbor_frames"].as<int>(3);
		g_global_ground_min_local_map_points = gr["min_local_map_points"].as<int>(4000);
		g_global_ground_min_map_ground_points = gr["min_map_ground_points"].as<int>(800);
		g_global_ground_min_frame_ground_points = gr["min_frame_ground_points"].as<int>(300);
		g_global_ground_min_residual_samples = gr["min_residual_samples"].as<int>(80);
		g_global_ground_max_pose_z_adjust_m = gr["max_pose_z_adjust_m"].as<float>(0.10f);
	}

	// --- Read optimized poses ---
	Values values;
	std::map<Key, double> timestamps;
	const std::vector<TumPose> tum_poses = readTumPose(input_tum);
	for (size_t i = 0; i < tum_poses.size(); ++i) {
		const Key key = static_cast<Key>(i);
		const Eigen::Quaterniond q = tum_poses[i].q.normalized();
		values.insert(key, gtsam::Pose3(
			gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
			gtsam::Point3(tum_poses[i].t.x(), tum_poses[i].t.y(), tum_poses[i].t.z())));
		timestamps[key] = tum_poses[i].timestamp;
	}
	std::cout << "[refined_ground] Loaded " << tum_poses.size() << " poses from " << input_tum << std::endl;

	if (values.empty()) {
		std::cerr << "[refined_ground] No poses loaded. Exiting." << std::endl;
		return -1;
	}

	// --- Ground Z refinement ---
	refinePoseZWithGlobalGround(values, timestamps, pointclouds_path);

	// --- Save refined poses ---
	saveValuesToTUM(values, timestamps, output_tum);

	std::cout << "[refined_ground] Done." << std::endl;
	return 0;
}


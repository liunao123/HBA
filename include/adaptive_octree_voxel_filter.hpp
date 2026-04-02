#pragma once
#include <map>
#include <utility>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <unordered_map>


template <typename PointT>
class AdaptiveOctreeVoxelFilter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using CloudT = pcl::PointCloud<PointT>;
    using CloudPtr = typename CloudT::Ptr;
    using PoseList = std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>;
    using PosePositionList = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

    CloudPtr raw_unified_cloud_;
    CloudPtr filtered_cloud_;

    explicit AdaptiveOctreeVoxelFilter(float octree_resolution = 5.0f)
        : octree_resolution_(octree_resolution),
          raw_unified_cloud_(new CloudT),
          filtered_cloud_(new CloudT) {
          std::cout << "AdaptiveOctreeVoxelFilter initialized with octree resolution: " << octree_resolution_ << std::endl;
        adaptive_voxel_params_ = {
            {0.0f, 0.025f},
            {10.0f, 0.05f},
            {20.0f, 0.1f},
            {30.0f, 0.2f},
            {FLT_MAX, 0.5f}
        };
    }

    bool setKeyPose(const PoseList& poses) {
        if (has_input_cloud_count_ && poses.size() != input_cloud_count_) {
            std::cerr << "[AdaptiveOctreeVoxelFilter] Size mismatch: poses=" << poses.size()
                      << ", clouds=" << input_cloud_count_ << std::endl;
            return false;
        }

        poses_ = poses;
        pose_positions_unified_.clear();
        has_origin_ = false;

        if (poses_.empty()) {
            origin_ = Eigen::Vector3d::Zero();
            return true;
        }

        origin_ = poses_.front().translation();
        has_origin_ = true;

        pose_positions_unified_.reserve(poses_.size());
        for (const auto& pose : poses_) {
            pose_positions_unified_.push_back(pose.translation());
        }

        input_pose_count_ = poses_.size();
        has_input_pose_count_ = true;
        return true;
    }

    bool setKeyPointCloud(const std::vector<CloudPtr>& clouds) {
        input_cloud_count_ = clouds.size();
        has_input_cloud_count_ = true;

        if (has_input_pose_count_ && clouds.size() != input_pose_count_) {
            std::cerr << "[AdaptiveOctreeVoxelFilter] Size mismatch: poses=" << input_pose_count_
                      << ", clouds=" << clouds.size() << std::endl;
            return false;
        }

        if (!has_origin_ || poses_.empty() || clouds.size() != poses_.size()) {
            if (!poses_.empty() && clouds.size() != poses_.size()) {
                std::cerr << "[AdaptiveOctreeVoxelFilter] Size mismatch: poses=" << poses_.size()
                          << ", clouds=" << clouds.size() << std::endl;
            }
            return false;
        }

        raw_unified_cloud_->clear();
        for (size_t i = 0; i < clouds.size(); ++i) {
            const auto& cloud = clouds[i];
            if (!cloud || cloud->empty()) continue;

            CloudT transformed_cloud;
            pcl::transformPointCloud(*cloud, transformed_cloud, poses_[i]);
            *raw_unified_cloud_ += transformed_cloud;
        }

        raw_unified_cloud_->width = raw_unified_cloud_->size();
        raw_unified_cloud_->height = 1;
        raw_unified_cloud_->is_dense = true;
        std::cout << "Unified cloud built with " << raw_unified_cloud_->size() << " points." << std::endl;
        return true;
    }

    void setAdaptiveVoxelParams(const std::vector<std::pair<float, float>>& adaptive_voxel_params) {
        adaptive_voxel_params_ = adaptive_voxel_params;
        std::sort(adaptive_voxel_params_.begin(), adaptive_voxel_params_.end(),
                  [](const std::pair<float, float>& lhs, const std::pair<float, float>& rhs) {
                      return lhs.first < rhs.first;
                  });
    }

    CloudPtr executeFiltering() {
        filtered_cloud_.reset(new CloudT);
        if (!raw_unified_cloud_ || raw_unified_cloud_->empty()) {
            return filtered_cloud_;
        }
        if (adaptive_voxel_params_.empty()) {
            return raw_unified_cloud_;
        }

        PointT minPt, maxPt;
        pcl::getMinMax3D(*raw_unified_cloud_, minPt, maxPt);

        const float x_range = maxPt.x - minPt.x;
        const float y_range = maxPt.y - minPt.y;
        const float z_range = maxPt.z - minPt.z;

        const int grid_x_num = std::max(1, static_cast<int>(std::ceil(x_range / octree_resolution_)));
        const int grid_y_num = std::max(1, static_cast<int>(std::ceil(y_range / octree_resolution_)));
        const int grid_z_num = std::max(1, static_cast<int>(std::ceil(z_range / octree_resolution_)));

        struct OctreeNode {
            CloudPtr cloud;
            float cx = 0.0f;
            float cy = 0.0f;
            float cz = 0.0f;
            bool has_pose = false;
            float min_pose_dist = FLT_MAX;

            OctreeNode() : cloud(new CloudT) {}
        };

        std::unordered_map<int, OctreeNode> octree_nodes;
        octree_nodes.reserve(std::max<size_t>(1, raw_unified_cloud_->size() / 1024));

        for (const auto& point : raw_unified_cloud_->points) {
            const int gx = std::clamp(static_cast<int>((point.x - minPt.x) / octree_resolution_), 0, grid_x_num - 1);
            const int gy = std::clamp(static_cast<int>((point.y - minPt.y) / octree_resolution_), 0, grid_y_num - 1);
            const int gz = std::clamp(static_cast<int>((point.z - minPt.z) / octree_resolution_), 0, grid_z_num - 1);
            const int idx = gz * grid_x_num * grid_y_num + gy * grid_x_num + gx;

            OctreeNode& node = octree_nodes[idx];
            if (node.cloud->empty()) {
                node.cx = minPt.x + (gx + 0.5f) * octree_resolution_;
                node.cy = minPt.y + (gy + 0.5f) * octree_resolution_;
                node.cz = minPt.z + (gz + 0.5f) * octree_resolution_;
            }
            node.cloud->push_back(point);
        }

        for (auto& kv : octree_nodes) {
            auto& node = kv.second;
            node.has_pose = false;
            node.min_pose_dist = FLT_MAX;

            for (const auto& pose_pos : pose_positions_unified_) {
                const float dx = static_cast<float>(pose_pos.x()) - node.cx;
                const float dy = static_cast<float>(pose_pos.y()) - node.cy;
                const float dist = std::sqrt(dx * dx + dy * dy);
                node.min_pose_dist = std::min(node.min_pose_dist, dist);

                const float half_res = octree_resolution_ * 0.5f;
                if (std::abs(dx) <= half_res && std::abs(dy) <= half_res) {
                    node.has_pose = true;
                }
            }
        }

        for (const auto& kv : octree_nodes) {
            const auto& node = kv.second;
            if (node.cloud->empty()) {
                continue;
            }

            float voxel_size = adaptive_voxel_params_.back().second;
            if (node.has_pose) {
                voxel_size = adaptive_voxel_params_[0].second;
            } else {
                for (size_t i = 1; i < adaptive_voxel_params_.size(); ++i) {
                    if (node.min_pose_dist < adaptive_voxel_params_[i].first) {
                        voxel_size = adaptive_voxel_params_[i].second;
                        break;
                    }
                }
            }

            CloudT filtered_node;
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(node.cloud);
            vg.setLeafSize(voxel_size, voxel_size, voxel_size);
            vg.filter(filtered_node);
            if (node.has_pose && filtered_node.size() < 20)
            {
                continue;
            }
            
            *filtered_cloud_ += filtered_node;
        }

        filtered_cloud_->width = static_cast<uint32_t>(filtered_cloud_->size());
        filtered_cloud_->height = 1;
        filtered_cloud_->is_dense = true;
        std::cout << "Filtered cloud has " << filtered_cloud_->size() << " points after adaptive voxel filtering." << std::endl;
        return filtered_cloud_;
    }

    CloudPtr getRawUnifiedCloud() const { return raw_unified_cloud_; }
    CloudPtr getFilteredUnifiedCloud() const { return filtered_cloud_; }

private:
    float octree_resolution_;
    bool has_origin_ = false;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    PoseList poses_;
    PosePositionList pose_positions_unified_;
    std::vector<std::pair<float, float>> adaptive_voxel_params_;
    size_t input_pose_count_ = 0;
    size_t input_cloud_count_ = 0;
    bool has_input_pose_count_ = false;
    bool has_input_cloud_count_ = false;
};
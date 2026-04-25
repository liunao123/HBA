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

        const int grid_x = std::max(1, static_cast<int>(std::ceil(x_range / octree_resolution_)));
        const int grid_y = std::max(1, static_cast<int>(std::ceil(y_range / octree_resolution_)));
        const int grid_z = std::max(1, static_cast<int>(std::ceil(z_range / octree_resolution_)));

        // ── Step 1: scan points once, build per-node metadata (no point copying) ──
        // 旧方案将所有点复制进 per-node CloudPtr，对 500M 点会导致内存翻倍 OOM。
        // 新方案只记录每个 octree cell 的中心坐标和 voxel_size，不存任何点。
        struct NodeMeta {
            float cx, cy, cz;
            float voxel_size = 0.0f;
            bool  has_pose   = false;
        };

        std::unordered_map<int32_t, NodeMeta> node_meta;
        {
            // 预估节点数上限，防止 overflow
            const size_t est = std::min(
                static_cast<size_t>(grid_x) * grid_y * grid_z / 2 + 16,
                static_cast<size_t>(4 * 1024 * 1024));
            node_meta.reserve(est);
        }

        const size_t N = raw_unified_cloud_->size();
        std::cout << "  [executeFiltering] Pass 1/2: node metadata (" << N << " pts)..." << std::flush;

        for (size_t pi = 0; pi < N; ++pi) {
            const auto& pt = raw_unified_cloud_->points[pi];
            const int gx = std::clamp(
                static_cast<int>((pt.x - minPt.x) / octree_resolution_), 0, grid_x - 1);
            const int gy = std::clamp(
                static_cast<int>((pt.y - minPt.y) / octree_resolution_), 0, grid_y - 1);
            const int gz = std::clamp(
                static_cast<int>((pt.z - minPt.z) / octree_resolution_), 0, grid_z - 1);
            const int32_t nid = gz * grid_x * grid_y + gy * grid_x + gx;

            if (!node_meta.count(nid)) {
                NodeMeta nm;
                nm.cx = minPt.x + (gx + 0.5f) * octree_resolution_;
                nm.cy = minPt.y + (gy + 0.5f) * octree_resolution_;
                nm.cz = minPt.z + (gz + 0.5f) * octree_resolution_;
                nm.voxel_size = adaptive_voxel_params_.back().second;
                node_meta.emplace(nid, nm);
            }
        }
        std::cout << " done. nodes=" << node_meta.size() << "\n";

        // 根据到最近轨迹点的距离，为每个节点确定 voxel_size
        const float half_res = octree_resolution_ * 0.5f;
        for (auto& kv : node_meta) {
            NodeMeta& nm = kv.second;
            float min_dist = FLT_MAX;
            for (const auto& pp : pose_positions_unified_) {
                const float dx = static_cast<float>(pp.x()) - nm.cx;
                const float dy = static_cast<float>(pp.y()) - nm.cy;
                const float d  = std::sqrt(dx * dx + dy * dy);
                if (d < min_dist) min_dist = d;
                if (std::abs(dx) <= half_res && std::abs(dy) <= half_res)
                    nm.has_pose = true;
            }
            if (nm.has_pose) {
                nm.voxel_size = adaptive_voxel_params_[0].second;
            } else {
                for (size_t i = 1; i < adaptive_voxel_params_.size(); ++i) {
                    if (min_dist < adaptive_voxel_params_[i].first) {
                        nm.voxel_size = adaptive_voxel_params_[i].second;
                        break;
                    }
                }
            }
        }

        // ── Step 2: 直接流式 voxel 哈希，无任何中间 per-node 点云 ─────────────
        // Key = (node_id, global_vx, global_vy, global_vz at local voxel_size)
        // 每个 voxel 只保留第一个遇到的点（等价于 VoxelGrid 的 centroid 近似）
        struct VoxelKey {
            int32_t nid, vx, vy, vz;
            bool operator==(const VoxelKey& o) const {
                return nid == o.nid && vx == o.vx && vy == o.vy && vz == o.vz;
            }
        };
        struct VoxelHash {
            size_t operator()(const VoxelKey& k) const noexcept {
                uint64_t h  = static_cast<uint64_t>(static_cast<uint32_t>(k.nid)) * 2654435761ULL;
                         h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.vx))  * 2246822519ULL;
                         h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.vy))  * 3266489917ULL;
                         h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.vz))  *  668265263ULL;
                return static_cast<size_t>(h ^ (h >> 32));
            }
        };

        std::unordered_map<VoxelKey, PointT, VoxelHash> voxel_map;
        voxel_map.reserve(1 << 20);  // 初始 1M buckets，按需自动扩容

        // 记录每个节点的输出点数，供 has_pose && size<20 过滤使用
        std::unordered_map<int32_t, uint32_t> node_out_count;
        node_out_count.reserve(node_meta.size());

        std::cout << "  [executeFiltering] Pass 2/2: voxel hashing..." << std::flush;
        const size_t step = std::max<size_t>(1, N / 20);  // 每 5% 打印一次进度

        for (size_t pi = 0; pi < N; ++pi) {
            if (pi % step == 0)
                std::cout << " " << pi * 100 / N << "%" << std::flush;

            const auto& pt = raw_unified_cloud_->points[pi];
            const int gx = std::clamp(
                static_cast<int>((pt.x - minPt.x) / octree_resolution_), 0, grid_x - 1);
            const int gy = std::clamp(
                static_cast<int>((pt.y - minPt.y) / octree_resolution_), 0, grid_y - 1);
            const int gz = std::clamp(
                static_cast<int>((pt.z - minPt.z) / octree_resolution_), 0, grid_z - 1);
            const int32_t nid = gz * grid_x * grid_y + gy * grid_x + gx;

            const auto mit = node_meta.find(nid);
            if (mit == node_meta.end()) continue;
            const float vs = mit->second.voxel_size;

            VoxelKey key{nid,
                static_cast<int32_t>(std::floor(pt.x / vs)),
                static_cast<int32_t>(std::floor(pt.y / vs)),
                static_cast<int32_t>(std::floor(pt.z / vs))};

            auto [it, inserted] = voxel_map.emplace(key, pt);
            if (inserted) ++node_out_count[nid];
        }
        std::cout << " done. output_voxels=" << voxel_map.size() << "\n";

        // 收集结果，保留 has_pose && size<20 过滤逻辑
        filtered_cloud_->reserve(voxel_map.size());
        for (auto& [key, point] : voxel_map) {
            const NodeMeta& nm = node_meta.at(key.nid);
            if (nm.has_pose && node_out_count[key.nid] < 20) continue;
            filtered_cloud_->push_back(std::move(point));
        }

        filtered_cloud_->width  = static_cast<uint32_t>(filtered_cloud_->size());
        filtered_cloud_->height = 1;
        filtered_cloud_->is_dense = true;
        std::cout << "Filtered cloud has " << filtered_cloud_->size()
                  << " points after adaptive voxel filtering." << std::endl;
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
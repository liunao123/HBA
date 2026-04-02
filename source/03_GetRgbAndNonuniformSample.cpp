#include "sensor.hpp"
#include <cmath>
#include <optional>
#include <utility>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

#include "common_func.cpp"
#include "common.hpp"

#include "adaptive_octree_voxel_filter.hpp"

using ProjectPointsResult = std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>;

std::optional<ProjectPointsResult> project_points_to_image(
    const cv::Mat &image,
    const cv::Mat &K,
    const cv::Mat &D,
    const Eigen::Matrix4d &T_cam_lidar,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    if (image.empty())
    {
        std::cerr << "[project_points_to_image] Input image is empty!" << std::endl;
        return std::nullopt;
    }
    if (K.empty() || K.rows != 3 || K.cols != 3)
    {
        std::cerr << "[project_points_to_image] Invalid camera intrinsic matrix K!" << std::endl;
        return std::nullopt;
    }
    if (!cloud || cloud->empty())
    {
        std::cerr << "[project_points_to_image] Input point cloud is empty!" << std::endl;
        return std::nullopt;
    }

    cv::Mat undistorted;
    if (D.empty())
    {
        undistorted = image.clone();
    }
    else
    {
        cv::undistort(image, undistorted, K, D);
    }
    cv::Mat overlay = undistorted.clone();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZI>);
    try
    {
        pcl::transformPointCloud(*cloud, *cloud_cam, T_cam_lidar.cast<float>());
    }
    catch (const std::exception &e)
    {
        std::cerr << "[project_points_to_image] Transform point cloud failed: " << e.what() << std::endl;
        return std::nullopt;
    }

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat distCoeffs = D.empty() ? cv::Mat::zeros(1, 5, CV_64F) : D.reshape(1, 1).clone();

    std::vector<cv::Point3f> pts3d(1);
    std::vector<cv::Point2f> pts2d(1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->reserve(cloud->size());

    for (size_t idx = 0; idx < cloud_cam->size(); ++idx)
    {
        const auto &pt_cam = cloud_cam->points[idx];
        const auto &pt_lidar = cloud->points[idx];

        if (!std::isfinite(pt_cam.x) || !std::isfinite(pt_cam.y) || !std::isfinite(pt_cam.z) || pt_cam.z <= 0.1f)
        {
            continue;
        }

        pts3d[0] = cv::Point3f(pt_cam.x, pt_cam.y, pt_cam.z);
        cv::projectPoints(pts3d, rvec, tvec, K, distCoeffs, pts2d);
        int px = static_cast<int>(std::round(pts2d[0].x));
        int py = static_cast<int>(std::round(pts2d[0].y));

        if (px < 2 || py < 2 || px >= overlay.cols - 2 || py >= overlay.rows - 2)
        {
            continue;
        }

        cv::Vec3b pixel_color = undistorted.at<cv::Vec3b>(py, px);
        pcl::PointXYZRGB colored_point;
        colored_point.x = pt_lidar.x;
        colored_point.y = pt_lidar.y;
        colored_point.z = pt_lidar.z;
        colored_point.r = pixel_color[2];
        colored_point.g = pixel_color[1];
        colored_point.b = pixel_color[0];
        colored_cloud->push_back(colored_point);

        float range_sq = pt_cam.x * pt_cam.x + pt_cam.y * pt_cam.y;
        const float max_range_sq = 20.0f * 20.0f;
        int red = std::min(255, static_cast<int>(255 * std::abs((range_sq - max_range_sq) / max_range_sq)));
        int green = std::min(255, static_cast<int>(255 * (1 - std::abs((range_sq - max_range_sq) / max_range_sq))));
        cv::circle(overlay, cv::Point(px, py), 1, cv::Scalar(0, green, red), -1);
    }

    colored_cloud->width = static_cast<uint32_t>(colored_cloud->size());
    colored_cloud->height = 1;
    colored_cloud->is_dense = false;

    return ProjectPointsResult{overlay, colored_cloud};
}

int main(int argc, char **argv)
{
    const std::string work_dir = "/home/xf/Desktop/catkin_ws/data/raw_data_20251220_1/";
    const std::string extrinsics_folder = work_dir + "/calib/extrinsics/";
    const std::string cam_folder = work_dir + "/calib/intrinsics/";
    const std::string lidar_name = "hesai128";
    const std::string image_root = work_dir + "/images";
    const std::string pcd_root = work_dir + "/pointclouds";
    const std::string pose_root = work_dir + "/spare/vehicle_geo_pose";
    const std::string rgb_pcd_root = work_dir + "/pointclouds_rgb";
    const std::string output_root = work_dir + "/spare/post";

    create_dir_if_not_exists(output_root);
    create_dir_if_not_exists(rgb_pcd_root);

    CameraIntrinsic cam_intr;
    cam_intr.loadFolder(cam_folder);

    ExtrinsicManager tf_cache;
    tf_cache.loadFolder(extrinsics_folder);

    std::vector<std::string> frame_id_timestamps = collect_frame_timestamps(pcd_root);
    if (frame_id_timestamps.empty())
    {
        std::cerr << "[Pipeline] No PCD files found in " << pcd_root << std::endl;
        return 1;
    }
    std::cerr << "[Pipeline] Found " << frame_id_timestamps.size() << " PCD files in " << pcd_root << std::endl;

    using FilterT = AdaptiveOctreeVoxelFilter<pcl::PointXYZRGB>;
    FilterT adaptive_filter(10.0f);
    FilterT::PoseList pose_list;
    std::vector<FilterT::CloudPtr> cloud_list;

    int processed_count = 0;

    for (const auto &frame : frame_id_timestamps)
    {
        processed_count++;
        const std::string pcd_path = pcd_root + "/" + frame + ".pcd";
        const std::string pose_path = pose_root + "/" + frame + ".yaml";

        // if (processed_count % 2 == 0)
        // {
        //     continue;
        // }

        if (processed_count % 100 == 0)
        {
            // std::cout << "[Pipeline] Processed 100 frames, stopping demo." << std::endl;
            std::cout << "[Pipeline] Processed " << processed_count << " frames." << std::endl;
            // break;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *original_cloud) == -1)
        {
            std::cerr << "[PCD] Failed to read: " << pcd_path << std::endl;
            continue;
        }

        // std::cout << "[Pipeline] original_cloud: " << original_cloud->size() << " points." << std::endl;
        const float min_m = 4.0f;

        pcl::CropBox<pcl::PointXYZI> region;                          
        region.setMin(Eigen::Vector4f(-min_m, -min_m, -100, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1.
        region.setMax(Eigen::Vector4f(min_m, min_m, 100, 1.0));
        region.setInputCloud(original_cloud); // 放入过滤后的点云
        region.setNegative(true);       // false 删除立方体内的点  
        region.filter(*cloud);         // 过滤，结果存入cloud
        // std::cout << "[Pipeline] Loaded and cropped cloud: " << cloud->size() << " points." << std::endl;

        // ====================== 修复 1：每一帧都创建新的点云！======================
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int cam_idx = 1; cam_idx <= 7; ++cam_idx)
        {
            const std::string camera_name = "cam" + std::to_string(cam_idx);
            const std::string image_path = image_root + "/" + camera_name + "/" + frame + ".jpg";

            cv::Mat K, D;
            int img_w = 0, img_h = 0;
            if (!cam_intr.getIntrinsic(camera_name, K, D, &img_w, &img_h))
            {
                std::cerr << "[cameraIntrinsic] Missing intrinsics for " << camera_name << std::endl;
                continue;
            }

            Eigen::Matrix4d T_cam_lidar;
            if (!tf_cache.lookupTransform(camera_name, lidar_name, T_cam_lidar))
            {
                std::cerr << "[TFCache] Missing transform: " << camera_name << " <- " << lidar_name << std::endl;
                continue;
            }

            cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
            if (image.empty())
            {
                std::cerr << "[Image] Failed to read: " << image_path << std::endl;
                continue;
            }

            if (img_w > 0 && img_h > 0 && (image.cols != img_w || image.rows != img_h))
                cv::resize(image, image, cv::Size(img_w, img_h));

            auto projection = project_points_to_image(image, K, D, T_cam_lidar, cloud);
            if (!projection)
            {
                std::cerr << "[Pipeline] Projection failed for " << frame << " " << camera_name << std::endl;
                continue;
            }
            *rgb_cloud += *(projection->second);

            // std::cout << "[Pipeline] Done: frame=" << frame << ", camera=" << camera_name << std::endl;
        }

        // ====================== 修复 2：累加后必须设置正确的云信息 ======================
        rgb_cloud->width = rgb_cloud->size();
        rgb_cloud->height = 1;
        rgb_cloud->is_dense = true;

        auto lidar_pose = readPoseFromYaml(pose_path);

        // pcl::transformPointCloud(*rgb_cloud, *rgb_cloud, lidar_pose.transformation_matrix);

        pose_list.push_back(lidar_pose.transformation_matrix);
        cloud_list.push_back(rgb_cloud);


        if ( processed_count % 100 )
            continue;

        const std::string final_colored_pcd = rgb_pcd_root + "/" + frame + ".pcd";
        if (pcl::io::savePCDFileBinary(final_colored_pcd, *rgb_cloud) == -1)
        {
            std::cerr << "[Pipeline] Failed to save final colored PCD: " << final_colored_pcd << std::endl;
        }
        else
        {
            std::cout << "[Pipeline] Saved final colored point cloud to: " << final_colored_pcd << std::endl;
        }

    }

    if (!adaptive_filter.setKeyPose(pose_list)) {
        std::cerr << "Failed to set key poses: pose/cloud vector size mismatch." << std::endl;
        return -1;
    }
    if (!adaptive_filter.setKeyPointCloud(cloud_list)) {
        std::cerr << "Failed to build unified cloud from batched poses/clouds." << std::endl;
        return -1;
    }

    auto raw_unified_cloud = adaptive_filter.getRawUnifiedCloud();
    std::cout << "local raw unified cloud " << raw_unified_cloud->size()   << std::endl;

    std::vector<std::pair<float, float>> adaptive_voxel_params = {
        {0.0f, 0.1f},
        {10.0f, 0.15f},
        {20.0f, 0.2f},
        {40.0f, 0.25f},
        {FLT_MAX, 0.3f}
    };
    adaptive_filter.setAdaptiveVoxelParams(adaptive_voxel_params);

    FilterT::CloudPtr merged_cloud;
    try
    {
        std::cout << "Start executeFiltering..." << std::endl;
        merged_cloud = adaptive_filter.executeFiltering();
        std::cout << "executeFiltering finished successfully!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    if (!merged_cloud || merged_cloud->empty())
    {
        std::cerr << "[Pipeline] Filtered cloud is empty." << std::endl;
    }
    else
    {
        const std::string merged_filename = output_root + "/filtered_map.pcd";
        pcl::io::savePCDFileBinary(merged_filename, *merged_cloud);
        std::cout << "Saved filtered cloud to: " << merged_filename << std::endl;
    }

    std::string raw_filename = output_root + "/raw_unfiltered_map.pcd";
    pcl::io::savePCDFileBinary(raw_filename, *raw_unified_cloud);
    std::cout << "Saved raw unified cloud to: " << raw_filename << std::endl;

    std::cout << "\n========================================" << std::endl;
    std::cout << "Processing completed!" << std::endl;
    std::cout << "Successfully processed: " << processed_count << " files" << std::endl;
    std::cout << "Raw points: " << raw_unified_cloud->size() << std::endl;
    std::cout << "Filtered points: " << (merged_cloud ? merged_cloud->size() : 0) << std::endl;
    std::cout << "Output directory: " << output_root << std::endl;

    return 0;
}
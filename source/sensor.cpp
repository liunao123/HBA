#include "sensor.hpp"
#include <cmath>
#include <optional>
#include <utility>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "common_func.cpp"

// ============================================================================
// Main Function (ROS Node Demo)
// ============================================================================

/**
 * @brief Main function demonstrating ROS node usage
 *
 * @param argc Argument count
 * @param argv Argument values
 * @return Exit code
 */

using ProjectPointsResult = std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>;

// ======================== 核心封装函数：project_points_to_image ========================
/**
 * @brief 将激光雷达点云投影到相机图像上，并生成彩色点云
 * @param image 输入的原始相机图像（未去畸变）
 * @param K 相机内参矩阵 (CV_64F)
 * @param D 相机畸变系数 (CV_64F，空则表示无畸变)
 * @param T_cam_lidar 雷达到相机的变换矩阵 (Eigen::Matrix4d)
 * @param cloud 输入的原始激光雷达点云 (pcl::PointXYZI)
 * @return 成功返回结果，失败返回std::nullopt
 */
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

    // 2. 图像去畸变
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

    // 3. 将点云转换到相机坐标系
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

    // 4. 准备cv::projectPoints参数
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); // 旋转向量（已转换到相机系，设为0）
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); // 平移向量（已转换到相机系，设为0）
    cv::Mat distCoeffs = D.empty() ? cv::Mat::zeros(1, 5, CV_64F) : D.reshape(1, 1).clone();

    std::vector<cv::Point3f> pts3d(1);
    std::vector<cv::Point2f> pts2d(1);

    // 5. 初始化彩色点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->reserve(cloud->size());

    // 6. 遍历点云，投影并绘制
    for (size_t idx = 0; idx < cloud_cam->size(); ++idx)
    {
        const auto &pt_cam = cloud_cam->points[idx];
        const auto &pt_lidar = cloud->points[idx];

        // 过滤无效点（非数值/相机后方点）
        if (!std::isfinite(pt_cam.x) || !std::isfinite(pt_cam.y) || !std::isfinite(pt_cam.z) || pt_cam.z <= 0.1f)
        {
            continue;
        }

        // 3D点投影到图像平面
        pts3d[0] = cv::Point3f(pt_cam.x, pt_cam.y, pt_cam.z);
        cv::projectPoints(pts3d, rvec, tvec, K, distCoeffs, pts2d);
        int px = static_cast<int>(std::round(pts2d[0].x));
        int py = static_cast<int>(std::round(pts2d[0].y));

        // 过滤图像外的点
        if (px < 2 || py < 2 || px >= overlay.cols - 2 || py >= overlay.rows - 2)
        {
            continue;
        }

        // 提取图像像素颜色，生成彩色点云（BGR转RGB）
        cv::Vec3b pixel_color = undistorted.at<cv::Vec3b>(py, px);
        pcl::PointXYZRGB colored_point;
        colored_point.x = pt_lidar.x;
        colored_point.y = pt_lidar.y;
        colored_point.z = pt_lidar.z;
        colored_point.r = pixel_color[2];
        colored_point.g = pixel_color[1];
        colored_point.b = pixel_color[0];
        colored_cloud->push_back(colored_point);

        // 绘制投影点（按距离平方上色）
        float range_sq = pt_cam.x * pt_cam.x + pt_cam.y * pt_cam.y;
        const float max_range_sq = 20.0f * 20.0f;
        int red = std::min(255, static_cast<int>(255 * std::abs((range_sq - max_range_sq) / max_range_sq)));
        int green = std::min(255, static_cast<int>(255 * (1 - std::abs((range_sq - max_range_sq) / max_range_sq))));
        cv::circle(overlay, cv::Point(px, py), 1, cv::Scalar(0, green, red), -1);
    }

    colored_cloud->width = static_cast<uint32_t>(colored_cloud->size());
    colored_cloud->height = 1;
    colored_cloud->is_dense = false;

    // if (!output_image.empty())
    // {
    //     if (!cv::imwrite(output_image, overlay))
    //     {
    //         std::cerr << "[project_points_to_image] Failed to save image to: " << output_image << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << "[project_points_to_image] Saved projection image to: " << output_image << std::endl;
    //     }
    // }

    // if (!colored_pcd.empty() && colored_cloud)
    // {
    //     if (pcl::io::savePCDFileBinary(colored_pcd, *colored_cloud) == -1)
    //     {
    //         std::cerr << "[project_points_to_image] Failed to save colored PCD to: " << colored_pcd << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << "[project_points_to_image] Saved colored point cloud to: " << colored_pcd << std::endl;
    //     }
    // }

    return ProjectPointsResult{overlay, colored_cloud};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_cache_demo");
    ros::NodeHandle nh;

    const std::string work_dir = "/media/xf/Elements/id4_1202/20251220_3/";
    const std::string extrinsics_folder = work_dir + "/calib/extrinsics/";
    const std::string cam_folder = work_dir + "/calib/intrinsics/";
    const std::string lidar_name = "hesai128";
    const std::string image_root = work_dir + "/images";
    const std::string pcd_root = work_dir + "/pointclouds_static";
    const std::string rgb_pcd_root = work_dir + "/pointclouds_rgb";
    const std::string output_root = "/home/xf/Desktop/catkin_ws/temp/";

    create_dir_if_not_exists(output_root);
    create_dir_if_not_exists(rgb_pcd_root);

    CameraIntrinsic cam_intr;
    cam_intr.loadFolder(cam_folder);

    ExtrinsicManager tf_cache;
    tf_cache.loadFolder(extrinsics_folder);

    // 指定需要处理的时间戳列表（可根据需求扩展或从文件读取）
    std::vector<std::string> frame_timestamps = collect_frame_timestamps(pcd_root);
    if (frame_timestamps.empty())
    {
        std::cerr << "[Pipeline] No PCD files found in " << pcd_root << std::endl;
        return 1;
    }

    for (const auto &frame : frame_timestamps)
    {
        const std::string pcd_path = pcd_root + "/" + frame + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1)
        {
            std::cerr << "[PCD] Failed to read: " << pcd_path << std::endl;
            continue;
        }
        // std::cerr << "[PCD] Failed to read: " << pcd_path << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int cam_idx = 1; cam_idx <= 7; ++cam_idx)
        {
            const std::string camera_name = "cam" + std::to_string(cam_idx);
            const std::string image_path = image_root + "/" + camera_name + "/" + frame + ".jpg";
            // const std::string image_path = image_root + "/" + camera_name + "/" + frame + "_" + camera_name + ".jpg";
            const std::string output_image = output_root + "/" + frame + "_" + camera_name + "_overlay.png";
            const std::string colored_pcd = output_root + "/" + frame + "_" + camera_name + "_colored.pcd";
            // std::cerr << "[image_path] Failed to read: " << image_path << std::endl;

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

            std::cout << "[Pipeline] Done: frame=" << frame << ", camera=" << camera_name << std::endl;
        }

        // 保存最终彩色点云
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

    return 0;
}
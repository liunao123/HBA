/**
 * @file Pose3SLAMExample_g2o.cpp modify by liunao 20250925
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the Pose3 using InitializePose3
 * Syntax for the script is ./Pose3SLAMExample_g2o input.g2o output.g2o [timestamps.tum]
 * The first pose is fixed during optimization. If timestamps.tum is provided, those timestamps will be used in output.
 * @date Aug 25, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>      // Robust核函数
#include <gtsam/linear/LossFunctions.h>   // mEstimator (Huber, Cauchy等)

#include <gtsam/geometry/Pose2.h>

#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <yaml-cpp/yaml.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
// PCL for GICP
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <yaml-cpp/yaml.h>

// Use nano_gicp instead of PCL GICP
#include <nano_gicp/nano_gicp.h>
// Common configuration and structures
#include "common.hpp"
#include "file_utils.hpp"

using namespace std;
using namespace gtsam;

typedef pcl::PointXYZI pointtype;
typedef pcl::PointCloud<pointtype>::Ptr cloud_ptr;

 


// Voxel filter parameters
float g_min_distance = 2.50;
float g_max_distance = 80.0;
float g_min_z = -2.30;
float g_max_z = 20.0;
float g_voxel_size = 0.15;

// GICP validation parameters
float g_max_trans_diff = 2.0;
float g_max_rot_diff = 0.5;

// GICP parameters
int g_gicp_correspondence_randomness = 128;
float g_gicp_max_correspondence_distance = 0.50;
int g_gicp_max_iterations = 128;
float g_gicp_transformation_epsilon = 0.001;
float g_gicp_rotation_epsilon = 0.001;
float g_gicp_initial_lambda_factor = 1e-9;

std::map<std::string, cloud_ptr> pointCloudCache;
std::mutex cacheMutex;

// 将优化后的 ENU 轨迹转到 UTM，并保存为 TUM 文件
std::vector<TumPose> to_utm_pose(const std::vector<TumPose> &opt_enu_poses, const std::string &temp_file_dir)
{
    if (opt_enu_poses.empty())
    {
        std::cerr << "to_utm_pose: empty opt_enu_poses, nothing to do" << std::endl;
    }

    // nongan
    // double lat0 = 43.99715149 , lon0 = 125.02480834, h0 = 201.48359680;

    // park
    // double lat0 = 31.426656703623681 , lon0 = 120.61981934083687, h0 = 13.115289688110352;

    std::string offset_file = temp_file_dir + "/utm_offset.yaml";
    std::cerr << "offset_file: " << offset_file << std::endl;

    // 1. 加载YAML文件（请替换为你的yaml文件实际路径）
    YAML::Node config = YAML::LoadFile(offset_file);

    // 3. 读取基础地理信息
    const double lat0 = config["lat"].as<double>();
    const double lon0 = config["lon"].as<double>();
    const double h0 = config["alt"].as<double>();

    // 4. 读取UTM偏移量（嵌套节点）
    YAML::Node utm_node = config["utm_offset"];
    const double utm_x0 = utm_node["x"].as<double>();
    const double utm_y0 = utm_node["y"].as<double>();

    std::cout << "to_utm_pose: ENU origin lat0=" << std::setprecision(8) << lat0
              << " lon0=" << lon0 << " h0=" << h0 << std::endl;

    GeographicLib::LocalCartesian local_cart(lat0, lon0, h0);

    std::string out_file = temp_file_dir + "opt_pose_utm.tum";
    std::ofstream fout(out_file);
    if (!fout.is_open())
    {
        std::cerr << "to_utm_pose: cannot open output file: " << out_file << std::endl;
    }

    std::vector<TumPose> utm_poses;
    utm_poses.reserve(opt_enu_poses.size());

    for (const auto &p_enu : opt_enu_poses)
    {
        // ENU -> 经纬高
        double lat, lon, h;
        local_cart.Reverse(p_enu.t.x(), p_enu.t.y(), p_enu.t.z(), lat, lon, h);
        // std::cerr << "124 lat, lon " << lat << ", " << lon << std::endl;

        // 经纬度 -> UTM
        int zone;
        bool northp;
        double utm_x, utm_y;
        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, utm_x, utm_y);
        // std::cerr << "131 utm_x " << utm_x << ", " << utm_y << std::endl;
        
        Eigen::Vector3d utm_t (utm_x - utm_x0, utm_y - utm_y0, h - h0);

            // 计算子午线收敛角
            bool utm_northp = true;
            int utm_zone = static_cast<int>(std::floor((lon + 180.0) / 6.0)) + 1;
            double lambda0 = utm_zone * 6.0 - 183.0; // UTM中央经线
            double delta_lambda = (lon - lambda0) * M_PI / 180.0;
            double phi = lat * M_PI / 180.0;
            // double gamma = std::atan(std::tan(delta_lambda) * std::sin(phi)); // 子午线收敛角，单位：弧度
            // Eigen::AngleAxisd rot_z(gamma, Eigen::Vector3d::UnitZ());

            double gamma = (delta_lambda) * std::sin(phi);
            double cos_gamma = std::cos(gamma);
            double sin_gamma = std::sin(gamma);
            
            Eigen::Matrix3d rot_z;
            rot_z << cos_gamma, -sin_gamma, 0,
                     sin_gamma,  cos_gamma, 0,
                     0,          0,         1;

            // pose转到UTM坐标系下
            Eigen::Matrix3d R_enu = p_enu.q.toRotationMatrix();
            // Eigen::Matrix3d R_utm = R_enu;
            Eigen::Matrix3d R_utm = rot_z * R_enu;

            Eigen::Quaterniond q_utm(R_utm);
            // ! ENU坐标转UTM坐标（只旋转，不平移）
            // ! todo 原始的位置就是 lidar在utm下的位置
            // Eigen::Vector3d t_utm = rot_z * (utm_t - opt_enu_poses[0].t) ;
            Eigen::Vector3d t_utm = utm_t; // + opt_enu_poses[0].t ;

            TumPose p_utm;
            p_utm.timestamp = p_enu.timestamp;
            p_utm.t = t_utm;
            p_utm.q = q_utm;
            utm_poses.push_back(p_utm);


        // 写 TUM：timestamp utm_x utm_y h qx qy qz qw
        fout << std::fixed << std::setprecision(3) << p_enu.timestamp << " "
            //  << std::setprecision(6) << utm_x - utm_x0 << " " << utm_y - utm_y0 << " " << h - h0 << " "
             << std::setprecision(6) << t_utm.x() << " " << t_utm.y() << " " << t_utm.z() << " "
             << q_utm.x() << " " << q_utm.y() << " " << q_utm.z() << " " << q_utm.w() << "\n";
    }

    fout.close();
    std::cout << "to_utm_pose: saved UTM trajectory to: " << out_file << std::endl;
    std::cout << "UTM trajectory size: " << utm_poses.size() << " poses" << std::endl;
    std::cout << "opt_enu_poses  size: " << opt_enu_poses.size() << " poses" << std::endl;
    return utm_poses;
}

void applyAdvancedVoxelFilter(const pcl::PointCloud<pointtype> &input_cloud,
                                            pcl::PointCloud<pointtype> &output_cloud)
{
    output_cloud.clear();
    output_cloud.reserve(input_cloud.size());

    // 针对拐弯重影问题的滤波参数优化
    const float min_distance = g_min_distance;  // 减小最小距离，保留近距离精确点
    const float max_distance = g_max_distance; // 减小最大距离，避免远距离噪声
    const float min_z = g_min_z;         // 收紧高度范围
    const float max_z = g_max_z;         // 收紧高度范围

    // 首先进行距离和高度滤波
    for (const auto &p : input_cloud.points)
    {
        if (std::abs(p.x) < min_distance && std::abs(p.y) < min_distance)
        {
            continue;
        }
        if (std::abs(p.x) > max_distance || std::abs(p.y) > max_distance)
        {
            continue;
        }

        if (p.z < min_z || p.z > max_z)
        {
            continue;
        }
        pcl::PointXYZI point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = p.intensity;
        output_cloud.points.push_back(point);
    }

    // std::cout << "Filter params - min_distance: " << min_distance << "m, max_distance: " << max_distance
    //           << "m, min_z: " << min_z << "m, max_z: " << max_z << "m" << std::endl;
    
    // std::cout << "Before voxel filter: " << output_cloud.size() << " points" << std::endl;
    // 应用体素网格滤波
    pcl::VoxelGrid<pointtype> voxel_grid;
    const float voxel_size = g_voxel_size;
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size); // 体素分辨率，适合高精度配准
    voxel_grid.setInputCloud(output_cloud.makeShared());
    voxel_grid.filter(output_cloud);
    // std::cout << "After voxel filter: " << output_cloud.size() << " points" << std::endl;
}

cloud_ptr getCachedPointCloud(std::string pcd_filename)
{
    // 检查缓存中是否已经有这个文件
    // {
    //     std::lock_guard<std::mutex> lock(cacheMutex);
    //     auto it = pointCloudCache.find(pcd_filename);
    //     if (it != pointCloudCache.end() && it->second)
    //     {
    //         // std::cout << "Using cached point cloud: " << pcd_filename 
    //         //           << " (" << it->second->size() << " points)" << std::endl;
    //         return it->second;
    //     }
    // }

    // 缓存中没有，需要加载
    cloud_ptr raw_cloud(new pcl::PointCloud<pointtype>());
    cloud_ptr cropped(new pcl::PointCloud<pointtype>());
    try
    {
        if (pcl::io::loadPCDFile<pointtype>(pcd_filename, *raw_cloud) == -1)
        {
            ROS_ERROR("Failed to load point cloud: %s", pcd_filename.c_str());
            // return nullptr;
        }
        // 首先进行距离和高度滤波
        const float crop_near_range = 5.0; // 剪裁近距离范围，单位：米
        for (const auto &p : raw_cloud->points)
        {
            if ( std::abs(p.x) < crop_near_range &&  std::abs(p.y) < crop_near_range)
            {
                continue;
            }
            if ( std::abs(p.x) > 100.0 || std::abs(p.y) > 100.0)
            {
                continue;
            }
            pcl::PointXYZI point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.intensity = p.intensity;
            cropped->points.push_back(point);
        }
        // for testing
        // pcl::io::savePCDFileBinary("cropped.pcd", *cropped);
        // exit(0);
        // raw_cloud->swap(*cropped);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    // std::cout << "Loaded point cloud: " << pcd_filename << " (" << cropped->size() << " points)" << std::endl;
    
    // cloud_ptr cloud_ds(new pcl::PointCloud<pointtype>());
    // applyAdvancedVoxelFilter(*cropped, *cloud_ds);

    // 必须要设置点云的基本属性
    cropped->width = cropped->points.size();
    cropped->height = 1;
    cropped->is_dense = raw_cloud->is_dense;

    // 存入缓存
    // std::lock_guard<std::mutex> lock(cacheMutex);
    // pointCloudCache[pcd_filename] = cropped;
    return cropped;
}



// Function to save trajectory in TUM format with optional timestamps
std::vector<TumPose>  saveTUMTrajectory(const Values &values, const string &filename, const map<Key, double> &timestamps = map<Key, double>() )
{
    cout << "Saving trajectory in TUM format to: " << filename << endl;

    ofstream file(filename);
    if (!file.is_open())
    {
        cerr << "Cannot open file: " << filename << endl;
    }

    // Write TUM format header
    // file << "# TUM trajectory format" << endl;
    // file << "# timestamp tx ty tz qx qy qz qw" << endl;

    // Sort poses by key for consistent output
    map<Key, Pose3> sorted_poses;
    for (const auto &key_value : values)
    {
        if (values.exists<Pose3>(key_value.key))
        {
            sorted_poses[key_value.key] = values.at<Pose3>(key_value.key);
        }
    }

    cout << "Writing " << sorted_poses.size() << " poses to TUM file..." << endl;
    bool using_external_timestamps = !timestamps.empty();
    if (using_external_timestamps)
    {
        cout << "Using external timestamps from TUM file" << endl;
    }
    else
    {
        cout << "Using key values as timestamps" << endl;
    }

    std::vector<TumPose> opt_poses;
    opt_poses.reserve(sorted_poses.size());

    for (const auto &pair : sorted_poses)
    {
        const Pose3 &pose = pair.second;
        Vector3 translation = pose.translation();
        gtsam::Quaternion rotation = pose.rotation().toQuaternion();

        // Use external timestamp if available, otherwise use key as timestamp
        double timestamp;
        if (using_external_timestamps && timestamps.find(pair.first) != timestamps.end())
        {
            timestamp = timestamps.at(pair.first);
        }
        else
        {
            timestamp = static_cast<double>(pair.first);
        }

        TumPose tp;
        tp.timestamp = timestamp;
        tp.t = Eigen::Vector3d(translation.x(), translation.y(), translation.z());
        tp.q = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        opt_poses.push_back(tp);

        // Write in TUM format: timestamp tx ty tz qx qy qz qw
        file << fixed << setprecision(3)
             << timestamp << " "
             << setprecision(6)
             << translation.x() << " " << translation.y() << " " << translation.z() << " "
             << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w()
             << endl;
    }

    file.close();
    cout << "Successfully saved TUM trajectory with " << sorted_poses.size() << " poses!" << endl;
    return opt_poses;
}

 

// Helper: convert gtsam::Pose3 to Eigen::Matrix4f
Eigen::Matrix4f pose3ToEigenMatrix4f(const gtsam::Pose3 &p)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    auto R = p.rotation().matrix();
    Eigen::Matrix3f Rf;
    Rf << static_cast<float>(R(0, 0)), static_cast<float>(R(0, 1)), static_cast<float>(R(0, 2)),
        static_cast<float>(R(1, 0)), static_cast<float>(R(1, 1)), static_cast<float>(R(1, 2)),
        static_cast<float>(R(2, 0)), static_cast<float>(R(2, 1)), static_cast<float>(R(2, 2));
    m.block<3, 3>(0, 0) = Rf;
    m(0, 3) = static_cast<float>(p.translation().x());
    m(1, 3) = static_cast<float>(p.translation().y());
    m(2, 3) = static_cast<float>(p.translation().z());
    return m;
}

// Helper: convert Eigen::Matrix4f to gtsam::Pose3
gtsam::Pose3 eigenMatrix4fToPose3(const Eigen::Matrix4f &m)
{
    Eigen::Matrix3d R;
    R << m(0, 0), m(0, 1), m(0, 2),
        m(1, 0), m(1, 1), m(1, 2),
        m(2, 0), m(2, 1), m(2, 2);
    Eigen::Vector3d t(m(0, 3), m(1, 3), m(2, 3));
    Eigen::Quaterniond q(R);
    return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()), gtsam::Point3(t.x(), t.y(), t.z()));
}

// load PCD filename of format id_time.pcd
cloud_ptr loadPCDForIdTimestamp(const std::string &pcd_dir, uint64_t id, double timestamp)
{
    std::ostringstream ss;
    ss << pcd_dir;
    if (!pcd_dir.empty() && pcd_dir.back() != '/' && pcd_dir.back() != '\\')
        ss << '/';
    // ss << std::fixed << std::setprecision(3) << timestamp << ".pcd";
    ss << id << "_" << std::fixed << std::setprecision(3) << timestamp << ".pcd";
    std::string filename = ss.str();
    // std::cout << "pcd filename: " << filename <<  std::endl;

    cloud_ptr cloud(new pcl::PointCloud<pointtype>());
    cloud = getCachedPointCloud(filename);
    // std::cout << "281 :::cloud->size() :  " << cloud->size() << " done " << std::endl;

    return cloud;
}


// For each pose in key_gnss_pose, find a pose in lio_pose with the same
// (or nearly the same) timestamp. Returned vector preserves the order of
// key_gnss_pose and contains the matching lio poses. If no match is found
// within `tol` seconds the entry is skipped and a warning is printed.
std::vector<TumPose> getSameTimePose(const std::vector<TumPose> &lio_pose,
                                  const std::vector<TumPose> &key_gnss_pose)
{
    std::vector<TumPose> matched;
    if (lio_pose.empty() || key_gnss_pose.empty())
        return matched;
    // For each GNSS key timestamp, find the closest LIO timestamp within tol
    for (const auto &kp : key_gnss_pose)
    {
        double kt = kp.timestamp;
        for (const auto &lp : lio_pose)
        {
            double dt = std::fabs(lp.timestamp - kt);
            if (dt == 0.0)
            {
                matched.push_back(lp);
                break;
            }
        }
    }
    return matched;
}

// Write a vector of TumPose back to a TUM-formatted file.
// This will reverse the coordinate/time adjustments made in readTumPose:
bool savePoseVectorToTUM(const std::vector<TumPose> &poses, const string &filename)
{
    if (poses.empty())
    {
        ROS_WARN("savePoseVectorToTUM: empty pose vector, nothing to write");
        return false;
    }

    std::ofstream file(filename);
    if (!file.is_open())
    {
        ROS_ERROR("savePoseVectorToTUM: cannot open file %s", filename.c_str());
        return false;
    }

    for (const auto &p : poses)
    {
        double out_ts = p.timestamp ; // + ts_offset;
        double tx = p.t.x();
        double ty = p.t.y();
        double tz = p.t.z();

        // Tum format: timestamp tx ty tz qx qy qz qw
        file << fixed << setprecision(3) << out_ts << " "
             << setprecision(6) << tx << " " << ty << " " << tz << " "
             << p.q.x() << " " << p.q.y() << " " << p.q.z() << " " << p.q.w()
             << "\n";
    }

    file.close();
    std::cout << "Saved " << poses.size() << " poses to TUM file: " << filename << std::endl;
    return true;
}


// Return a subsampled list of GNSS poses such that consecutive returned poses
// are at least `min_dist` meters apart OR yaw angle differs by at least `min_yaw_deg` degrees.
// Keeps the first pose and then greedily selects subsequent poses that satisfy either condition.
std::vector<TumPose> getKeyGnssPose(const std::vector<TumPose> &gnss_pose, double min_dist = 1.0, double min_yaw_deg = 5.0)
{
    std::vector<TumPose> keys;
    if (gnss_pose.empty())
        return keys;

    keys.push_back(gnss_pose.front());
    Eigen::Vector3d last_pos = gnss_pose.front().t;
    Eigen::Quaterniond last_quat = gnss_pose.front().q;

    // Convert min_yaw_deg to radians
    const double min_yaw_rad = min_yaw_deg * M_PI / 180.0;

    for (size_t i = 1; i < gnss_pose.size(); ++i)
    {
        const Eigen::Vector3d &pos = gnss_pose[i].t;
        const Eigen::Quaterniond &quat = gnss_pose[i].q;

        // Check distance threshold
        double dist = (pos - last_pos).norm();
        
        // Check yaw angle difference
        // Extract yaw from quaternions
        auto getYaw = [](const Eigen::Quaterniond &q) -> double {
            // Convert quaternion to Euler angles (roll, pitch, yaw)
            // yaw = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()))
            return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                             1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        };
        
        double yaw_current = getYaw(quat);
        double yaw_last = getYaw(last_quat);
        double yaw_diff = std::abs(yaw_current - yaw_last);
        
        // Normalize yaw difference to [-pi, pi]
        while (yaw_diff > M_PI) {
            yaw_diff -= 2.0 * M_PI;
        }
        yaw_diff = std::abs(yaw_diff);

        // Keep the pose if either distance OR yaw condition is met
        if (dist >= min_dist || yaw_diff >= min_yaw_rad)
        {
            keys.push_back(gnss_pose[i]);
            last_pos = pos;
            last_quat = quat;
            
            std::cout << "Key frame " << i << ": dist=" << dist << "m, yaw_diff=" 
                      << (yaw_diff * 180.0 / M_PI) << "° (kept by " 
                      << (dist >= min_dist ? "distance" : "yaw") << ")" << std::endl;
        }
    }
    
    std::cout << "Selected " << keys.size() << " key frames from " << gnss_pose.size() 
              << " total frames (min_dist=" << min_dist << "m, min_yaw=" << min_yaw_deg << "°)" << std::endl;
    
    return keys;
}


// Run GICP between two clouds, return transform mapping source->target and fitness score
bool runGICPGetRelative(const cloud_ptr &target,
                        const cloud_ptr &source,
                        const Eigen::Matrix4f &init_guess,
                        Eigen::Matrix4f &out_T,
                        double &out_fitness,
                        int max_iter = 50 )
{
    // basic null checks
    if (!target || !source)
    {
        ROS_WARN("runGICPGetRelative: null cloud pointer");
        return false;
    }

    // validate init_guess (finite)
    bool finite = true;
    for (int r = 0; r < 4 && finite; ++r)
        for (int c = 0; c < 4 && finite; ++c)
            if (!std::isfinite(init_guess(r, c)))
                finite = false;
    if (!finite)
    {
        ROS_WARN("runGICPGetRelative: init_guess contains non-finite values");
        return false;
    }

    ROS_INFO("runGICP: target_ds=%zu source_ds=%zu (voxel=%.3f)", target->size(), source->size(), g_voxel_size);
    cloud_ptr target_ds(new pcl::PointCloud<pointtype>());
    cloud_ptr source_ds(new pcl::PointCloud<pointtype>());
    applyAdvancedVoxelFilter(*source, *source_ds);
    applyAdvancedVoxelFilter(*target, *target_ds);
    ROS_INFO("runGICP: target_ds=%zu source_ds=%zu (voxel=%.3f)", target_ds->size(), source_ds->size(), g_voxel_size);

    nano_gicp::NanoGICP<pointtype, pointtype> gicp;
    gicp.setCorrespondenceRandomness(g_gicp_correspondence_randomness);
    gicp.setMaxCorrespondenceDistance(g_gicp_max_correspondence_distance);
    gicp.setMaximumIterations(g_gicp_max_iterations);
    gicp.setTransformationEpsilon(g_gicp_transformation_epsilon);
    gicp.setRotationEpsilon(g_gicp_rotation_epsilon);
    gicp.setInitialLambdaFactor(g_gicp_initial_lambda_factor);
    gicp.setRegularizationMethod(nano_gicp::RegularizationMethod::PLANE);
    gicp.setInputSource(source_ds);
    gicp.setInputTarget(target_ds);
    gicp.calculateSourceCovariances();
    gicp.calculateTargetCovariances();

    pcl::PointCloud<pointtype> Final;

    gicp.align(Final, init_guess);

    if (!gicp.hasConverged())
    {
        ROS_INFO("GICP did not converge (fitness=%f)", gicp.getFitnessScore());
        return false;
    }
    Eigen::Matrix4f final_tf = gicp.getFinalTransformation();
    out_T = final_tf;
    out_fitness = gicp.getFitnessScore();
    
    // ROS_INFO("GICP converged fitness=%f", out_fitness);
    
    static int cnt = 0;
    std::string savePCDDirectory = "./loop_closure/";
    mkdir((savePCDDirectory).c_str(), 0777);
    if (cnt % 50 == 0)
    {
        std::cout << "debug file save to savePCDDirectory: " << savePCDDirectory << std::endl;   
        pcl::io::savePCDFileBinary(savePCDDirectory + std::to_string(cnt) + "unused_result.pcd", Final);
        pcl::io::savePCDFileBinary(savePCDDirectory + std::to_string(cnt) + "prevKeyframeCloud.pcd", *target);
        pcl::io::savePCDFileBinary(savePCDDirectory + std::to_string(cnt) + "cureKeyframeCloud.pcd_" + std::to_string(out_fitness), *source);
        // 判断final_tf与init_guess的差异，若过大则认为未收敛
        std::cout << "init_guess: \n" << init_guess << std::endl;   
        std::cout << "final_tf: \n" << final_tf << std::endl;   
    }
    cnt++;
    return true;
} 

// Add loop closures between poses based on time and spatial thresholds using GICP on PCDs
void addLoopToGraph(NonlinearFactorGraph &graph,
                    const Values &initial,
                    const map<Key, double> &timestamps,
                    const std::string &pcd_dir,
                    const LoopConfig &loop_config )
{
    // collect keys in initial
    std::vector<Key> keys;
    for (const auto &kv : initial)
        keys.push_back(kv.key);
    std::sort(keys.begin(), keys.end());

    const int step = loop_config.step;
    int max_loop_index = loop_config.end_index - step;

    if ( loop_config.end_index >= keys.size() )
    {
        ROS_WARN("addLoopToGraph: invalid loop_config start_index or end_index");
        max_loop_index = keys.size() - step ;
        // exit(-1);
    }
    ROS_INFO("addLoopToGraph:  end_index=%d", max_loop_index);

    // for (size_t i = 0; i < keys.size()  ; i = i + step)
    for (size_t i = loop_config.start_index; i < max_loop_index ; i = i + step)
    {
        Key ki = keys[i];
        if (!initial.exists<Pose3>(ki))
            continue;
        Pose3 pi = initial.at<Pose3>(ki);
        double ti = 0;
        auto it_ti = timestamps.find(ki);
        if (it_ti != timestamps.end())
            ti = it_ti->second;

        auto cloud_i = loadPCDForIdTimestamp(pcd_dir, ki, ti);
        if ( !cloud_i ) continue;

        for (size_t j = max_loop_index; j >  step ; j = j - step)
        {   // skip adjacent (i+1)
            Key kj = keys[j];
            if (!initial.exists<Pose3>(kj))
                continue;
            Pose3 pj = initial.at<Pose3>(kj);
            double tj = 0;
            auto it_tj = timestamps.find(kj);
            if (it_tj != timestamps.end())
                tj = it_tj->second;

            if (std::fabs(tj - ti) < loop_config.time_thresh)
                continue;
            if (std::fabs(kj - ki) < loop_config.min_key_diff)
                continue;

            // spatial distance
            Eigen::Vector3d di(pi.translation().x() - pj.translation().x(),
                               pi.translation().y() - pj.translation().y(),
                               pi.translation().z() - pj.translation().z());
            double dist = di.norm();
            if (dist > loop_config.spatial_thresh)
                continue;

            // load PCDs
            if (timestamps.empty())
            {
                ROS_WARN("Timestamps empty, cannot load PCD by id_time");
                continue;
            }
            auto cloud_j = loadPCDForIdTimestamp(pcd_dir, kj, tj);
            if (!cloud_i || !cloud_j)
                continue;

            Eigen::Matrix4f init_guess = pose3ToEigenMatrix4f(pi.inverse() * pj);
            Eigen::Matrix4f Tij;
            double fitness = 999.0;
            // std::cout << "Running GICP between " << ki << " and " << kj << std::endl;
            bool ok = runGICPGetRelative(cloud_i, cloud_j, init_guess, Tij, fitness, loop_config.max_iter );
            if (!ok)
                continue;

            // ⚠️ 严格的fitness阈值过滤: 拒绝质量差的loop closure，避免引入错误约束
            // if (fitness > loop_config.max_fitness_reject) {
            //     std::cout << "❌ Rejected loop " << ki << "-" << kj << " fitness=" << fitness 
            //               << " (threshold=" << loop_config.max_fitness_reject << ")" << std::endl;
            //     continue;
            // }

            Pose3 meas = eigenMatrix4fToPose3(Tij);
            
            // ========== Loop Closure权重调整 (修正重影问题) ==========
            // 问题诊断: 单独loop匹配正常，但优化后仍有重影
            // 原因: odom约束太强，loop约束太弱，无法充分修正重影
            // 解决: 增强loop约束，让它能够有效拉齐重影点云
            // 策略: loop约束应该强于odom，这样才能修正累积漂移造成的重影
            
            double var_trans, var_rot;
            if (fitness < 0.5) {
                // 优质匹配: 强约束，可信赖，用于消除重影
                var_trans = loop_config.var_trans;   // 从配置读取
                var_rot = loop_config.var_rot;    // 从配置读取
                std::cout << "✅ High quality loop (fitness=" << fitness << ") - Strong constraint" << std::endl;
            } else {
                // var_trans = 2 * loop_config.var_trans;   // 从配置读取
                // var_rot = 2 * loop_config.var_rot;    // 从配置读取
                // 一般匹配: 中等约束，基于fitness动态调整
                var_trans = std::max(2.0, fitness );  
                var_rot = std::max(0.1, fitness * 0.1);
                var_trans = std::min(var_trans, 0.10);  // 最大10m
                var_rot = std::min(var_rot, 0.005);       // 最大0.5rad
                std::cout << "⚠️  Normal quality loop (fitness=" << fitness << ") - Medium constraint" << std::endl;
            }

            var_trans = var_trans * var_trans;
            var_rot = var_rot * var_rot;

            gtsam::Vector variances = (gtsam::Vector(6) << var_rot, var_rot, var_rot, var_trans, var_trans, var_trans).finished();
            auto noise = noiseModel::Diagonal::Variances(variances);
            graph.add( BetweenFactor<Pose3>(ki, kj, meas, noise) );
            std::cout << "Added loop constraint between " << ki << " and " << kj 
                      << " dist=" << dist << " time_diff=" << std::fabs(tj - ti) 
                      << " fitness=" << fitness 
                      << " trans_std=" << std::sqrt(var_trans) << "m"
                      << " rot_std=" << std::sqrt(var_rot) << "rad" << std::endl;
            // ROS_ERROR("Added loop constraint between %d and %d : fitness=%f, trans_std=%.2fm", 
            //           ki, kj, fitness, std::sqrt(var_trans));
            // 保存到vector
            // if (loop_constraints_out) {
            //     LoopConstraint lc{ki, kj, meas, variances};
            //     loop_constraints_out->push_back(lc);
            // }
            // break;
        }
    }
}


// 读loop closure约束g2o-like文本文件并加到graph
bool readLoopClosuresFromG2O(const std::string& filename, NonlinearFactorGraph& graph) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open loop g2o file: " << filename << std::endl;
        return false;
    }
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string tag;
        Key ki, kj;
        double tx, ty, tz, qx, qy, qz, qw;
        double var_rot, var_rot2, var_rot3, var_trans, var_trans2, var_trans3;
        iss >> tag >> ki >> kj >> tx >> ty >> tz >> qx >> qy >> qz >> qw >> var_rot >> var_rot2 >> var_rot3 >> var_trans >> var_trans2 >> var_trans3;
        gtsam::Pose3 meas(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
        gtsam::Vector variances = (gtsam::Vector(6) << var_rot, var_rot2, var_rot3, var_trans, var_trans2, var_trans3).finished();
        auto noise = noiseModel::Diagonal::Variances(variances / 10.0);
        graph.add(BetweenFactor<Pose3>(ki, kj, meas, noise));
        ++count;
    }
    std::cout << "Loaded " << count << " loop closures from " << filename << std::endl;
    return true;
}


void pts_to_world(const cloud_ptr &pts_local,
                        cloud_ptr &pts_world,
                        const TumPose &pose )
{
    if (!pts_local)
    {
        ROS_WARN("pts_to_world: pts_local is null");
        return;
    }
    if (pts_local->empty())
    {
        // nothing to do
        if (!pts_world)
            pts_world.reset(new pcl::PointCloud<pointtype>());
        pts_world->clear();
        return;
    }

    if (!pts_world)
        pts_world.reset(new pcl::PointCloud<pointtype>());

    // PCL transformPointCloud expects float transform matrix (Matrix4f/Affine3f)
    Eigen::Matrix4f key_pose = Eigen::Matrix4f::Identity();
    key_pose.block<3, 3>(0, 0) = pose.q.toRotationMatrix().cast<float>();
    key_pose.block<3, 1>(0, 3) = pose.t.cast<float>();

    try
    {
        pcl::transformPointCloud(*pts_local, *pts_world, key_pose);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("pts_to_world: exception during point cloud transformation: %s", e.what());
        pts_world->clear();
    }
}

void save_map_grid(std::string tile_output_dir, const cloud_ptr &transformedCloud, const std::vector<TumPose> &key_poses)
{
    {
      std::string tile_dir = tile_output_dir;
      if (!tile_dir.empty() && tile_dir.back() != '/' )
      {
        tile_dir += "/";
      }
      
      if (!tile_dir.empty())
      {
        bool tile_dir_ready = true;
        struct stat st;
        if (stat(tile_dir.c_str(), &st) != 0)
        {
          if (mkdir(tile_dir.c_str(), 0777) != 0)
        {
          ROS_ERROR("Failed to create tile directory: %s", tile_dir.c_str());
          tile_dir_ready = false;
        }
        }
      
        if (tile_dir_ready)
        {
          std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr> tile_clouds;
          const double tile_size = 100.0;
          
          // Ensure tile_x and tile_y are positive and start from 0 using PCL's library
          pcl::PointCloud<pcl::PointXYZ> cloud;
          for (const auto& pose : key_poses) {
              cloud.push_back(pcl::PointXYZ(pose.t.x(), pose.t.y(), 0));
          }
          pcl::PointXYZ min_pt, max_pt;
          pcl::getMinMax3D(cloud, min_pt, max_pt);
          // 保证比点云的最小值还要小一些，避免边界点落在负tile索引
          double min_x = min_pt.x - 200.0;
          double min_y = min_pt.y - 200.0;
          for (const auto &pt : transformedCloud->points)
          {
            int tile_x = static_cast<int>(std::floor((pt.x - min_x) / tile_size));
            int tile_y = static_cast<int>(std::floor((pt.y - min_y) / tile_size));
            std::string key = std::to_string(tile_x) + "_" + std::to_string(tile_y);
            auto &cloud_ptr = tile_clouds[key];
            if (!cloud_ptr)
            {
              cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
            }
            cloud_ptr->points.push_back(pt);
          }
          
          for (auto &kv : tile_clouds)
          {
            if (kv.second->empty())
            continue;
            std::string filename = tile_dir + "global_map_tile_" + kv.first + ".pcd";
            kv.second->width = kv.second->points.size();
            kv.second->height = 1;
            // std::cout << "kv.second->is_dense: " << kv.second->is_dense << std::endl;
            kv.second->is_dense = true ; //默认值
            pcl::io::savePCDFileBinary(filename, *kv.second);
            ROS_INFO("Saved tile %s (%zu points)", filename.c_str(), kv.second->points.size());
          }
        }
      }
    }
    ROS_WARN("save all points done . " );
}

int main(int argc, char **argv)
{
    // Initialize ROS for parameter reading
    ros::init(argc, argv, "pose3_slam_g2o");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Load YAML configuration file
    std::string config_file = "/home/xf/Desktop/catkin_ws/src/HBA/rviz_cfg/config.yaml";
    // pnh.param<std::string>("config_file", config_file, "./config.yaml");
    std::cout << "config_file :" << config_file << std::endl;
    
    // Configuration variables
    std::string work_dir, lio_tum_pose;
    std::string output_tum_file, pcd_dir, optimization_method;
    NoiseConfig noise_config;
    LoopConfig loop_config;
    LidarGnssExtrinsic extrinsic;
    bool save_loop_g2o, load_loop_g2o;
    YAML::Node config;
    
    // Load all configuration from YAML
    loadConfigFromYAML(config_file, work_dir, optimization_method,
                       noise_config, loop_config, save_loop_g2o, load_loop_g2o, extrinsic, config);

    // Get all YAML files from gnss_odoms_path directory
    std::string gnss_odoms_path =  work_dir + "/odoms";
    std::string pointclouds_path = work_dir + "/pointclouds";
    
    std::string temp_file_dir = work_dir + "/debug_file/";
    std::string output_g2o_file = temp_file_dir + "pose_graph.g2o";
    std::string opt_tum_file = temp_file_dir + "opt_pose_enu.tum";

    std::vector<std::string> gnss_odom_files = getFilesWithExtension(gnss_odoms_path, ".yaml");
    
    // Read all GNSS odom data from YAML files
    std::vector<GnssOdomData> gnss_odom_data = readAllGnssOdomYamls(gnss_odom_files);
    std::cout << "Loaded " << gnss_odom_data.size() << " GNSS odom data entries" << std::endl;
    if (!gnss_odom_data.empty()) {
        std::cout << "First entry - timestamp: " << gnss_odom_data[10].timestamp 
        << ", position: [" << gnss_odom_data[10].position.transpose() << "]"
        << ", heading: " << gnss_odom_data[10].heading << "°" << std::endl;
    }
    else
    {
        std::cerr << "No GNSS odom data loaded. Exiting." << std::endl;
        return -1;
    }
    // return -1;
    
    // Convert GNSS odom data to LiDAR poses using extrinsic calibration
    std::vector<TumPose> lidar_poses = GetLidarPoseOnWorld(gnss_odom_data, extrinsic);

    // 新增：允许通过指定TUM文件直接读取相邻帧的位姿作为里程计约束，跳过GICP
    std::string tum_odom_file =  work_dir + "/debug_file/opt_pose_enu1.tum";

    // if (config["odom_tum_file"]) {
    //     tum_odom_file = config["odom_tum_file"].as<std::string>("");
    // }
    // if (argc > 3) {
    //     tum_odom_file = argv[3];
    // }

    // Build graph and initial estimate from TUM poses
    NonlinearFactorGraph graph;
    Values initial;
    std::map<Key, double> key_frame_timestamps;

    // Add initial poses to Values
    std::cout << "\n========== Adding initial poses to graph ==========" << std::endl;
    for (size_t i = 0; i < lidar_poses.size(); ++i) {
        const TumPose &pose = lidar_poses[i];
        key_frame_timestamps.insert({static_cast<Key>(i), pose.timestamp});
        gtsam::Quaternion q(pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z());
        gtsam::Rot3 R = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
        gtsam::Point3 t(pose.t.x(), pose.t.y(), pose.t.z());
        gtsam::Pose3 curr(R, t);
        Key key = static_cast<Key>(i);
        initial.insert(key, curr);
        // Add prior for first pose
        if (i == 0) {
        // if (i == 0 or i == lidar_poses.size()-1 ) {
            gtsam::Vector priorVars = (gtsam::Vector(6) << 1e4, 1e4, 1e4, 1e-6, 1e-6, 1e-6).finished();
            auto priorNoise = noiseModel::Diagonal::Variances(priorVars);
            graph.add(PriorFactor<Pose3>(key, curr, priorNoise));
            std::cout << "Added strong prior for first pose (key=" << key << ")" << std::endl;
        }
        // Add prior factor every 100 poses
        // else {
        else if (i % 50 == 0) {
            gtsam::Vector priorVars100 = (gtsam::Vector(6) << 1e4, 1e4, 1e4, 4e-4, 4e-4, 9e-4).finished();
            auto priorNoise100 = noiseModel::Diagonal::Variances(priorVars100);
            graph.add(PriorFactor<Pose3>(key, curr, priorNoise100));
            std::cout << "Added periodic prior for pose " << i << " (key=" << key << ")" << std::endl;
        }
    }
    std::cout << "Added " << lidar_poses.size() << " initial poses" << std::endl;

    // Define noise model for GICP-based odometry constraints
    double gicp_rot_std = noise_config.odom_rot_std;
    double gicp_trans_std_xy = noise_config.odom_trans_std_xy;
    double gicp_trans_std_z = noise_config.odom_trans_std_z;
    gtsam::Vector gicpVars = (gtsam::Vector(6) << gicp_rot_std * gicp_rot_std,
                                                   gicp_rot_std * gicp_rot_std,
                                                   gicp_rot_std * gicp_rot_std,
                                                   gicp_trans_std_xy * gicp_trans_std_xy,
                                                   gicp_trans_std_xy * gicp_trans_std_xy,
                                                   gicp_trans_std_z * gicp_trans_std_z).finished();
    auto gicpNoise = noiseModel::Diagonal::Variances(gicpVars);

    if (!tum_odom_file.empty()) {
    // if (0) {
        // 直接从TUM文件读取相邻帧的位姿，构建里程计约束
        std::cout << "[INFO] Using TUM file for odometry constraints: " << tum_odom_file << std::endl;
        std::vector<TumPose> tum_odoms = readTumPose(tum_odom_file );
        if (tum_odoms.size() < 2) {
            std::cerr << "[ERROR] Not enough poses in TUM file for odometry constraints." << std::endl;
            return -1;
        }
        for (size_t i = 0; i < tum_odoms.size() - 1; ++i) {
            const TumPose &pose_i = tum_odoms[i];
            const TumPose &pose_j = tum_odoms[i + 1];
            gtsam::Quaternion qi(pose_i.q.w(), pose_i.q.x(), pose_i.q.y(), pose_i.q.z());
            gtsam::Rot3 Ri = gtsam::Rot3::Quaternion(qi.w(), qi.x(), qi.y(), qi.z());
            gtsam::Point3 ti(pose_i.t.x(), pose_i.t.y(), pose_i.t.z());
            gtsam::Pose3 pi(Ri, ti);
            gtsam::Quaternion qj(pose_j.q.w(), pose_j.q.x(), pose_j.q.y(), pose_j.q.z());
            gtsam::Rot3 Rj = gtsam::Rot3::Quaternion(qj.w(), qj.x(), qj.y(), qj.z());
            gtsam::Point3 tj(pose_j.t.x(), pose_j.t.y(), pose_j.t.z());
            gtsam::Pose3 pj(Rj, tj);
            gtsam::Pose3 rel = pi.inverse() * pj;
            Key ki = static_cast<Key>(i);
            Key kj = static_cast<Key>(i + 1);
            graph.add(BetweenFactor<Pose3>(ki, kj, rel, gicpNoise));
        }
        std::cout << "[INFO] Added " << (tum_odoms.size() - 1) << " odometry constraints from TUM file." << std::endl;
    } else {
        // ...existing code for GICP odometry constraints...
        std::vector<std::string> pcd_files = convertYamlPathsToPcdPaths(gnss_odom_files, pointclouds_path);
        // std::cout << "  pcd_files.size()  " <<  pcd_files.size() << std::endl;
        // std::cout << "  pcd_files.size()  " <<  pcd_files.back() << std::endl;
        int successful_gicp = 0;
        int failed_gicp = 0;
        for (size_t i = 0; i < pcd_files.size() - 1; ++i) {
            const TumPose &pose_i = lidar_poses[i];
            const TumPose &pose_j = lidar_poses[i + 1];
            cloud_ptr cloud_i = getCachedPointCloud(pcd_files[i]);
            cloud_ptr cloud_j = getCachedPointCloud(pcd_files[i + 1]);
            if (!cloud_i || !cloud_j || cloud_i->empty() || cloud_j->empty()) {
                std::cerr << "Failed to load point clouds for frames " << i << " and " << (i+1) << std::endl;
                failed_gicp++;
                continue;
            }
            gtsam::Quaternion qi(pose_i.q.w(), pose_i.q.x(), pose_i.q.y(), pose_i.q.z());
            gtsam::Rot3 Ri = gtsam::Rot3::Quaternion(qi.w(), qi.x(), qi.y(), qi.z());
            gtsam::Point3 ti(pose_i.t.x(), pose_i.t.y(), pose_i.t.z());
            gtsam::Pose3 pi(Ri, ti);
            gtsam::Quaternion qj(pose_j.q.w(), pose_j.q.x(), pose_j.q.y(), pose_j.q.z());
            gtsam::Rot3 Rj = gtsam::Rot3::Quaternion(qj.w(), qj.x(), qj.y(), qj.z());
            gtsam::Point3 tj(pose_j.t.x(), pose_j.t.y(), pose_j.t.z());
            gtsam::Pose3 pj(Rj, tj);
            Eigen::Matrix4f init_guess = pose3ToEigenMatrix4f(pi.inverse() * pj);
            std::cerr << "try GICP for frames " << i << " -> " << (i+1) << std::endl;
            Eigen::Matrix4f Tij;
            double fitness = 999.0;
            bool ok = runGICPGetRelative(cloud_i, cloud_j, init_guess, Tij, fitness, loop_config.max_iter);
            if (!ok) {
                failed_gicp++;
                continue;
            }
            gtsam::Pose3 meas = eigenMatrix4fToPose3(Tij);
            Key ki = static_cast<Key>(i);
            Key kj = static_cast<Key>(i + 1);
            graph.add(BetweenFactor<Pose3>(ki, kj, meas, gicpNoise));
            successful_gicp++;
            if ((i + 1) % 10 == 0) {
                std::cout << "Processed " << (i + 1) << "/" << (pcd_files.size() - 1) 
                          << " frame pairs (fitness=" << fitness << ")" << std::endl;
            }
        }
        std::cout << "\n========== GICP Odometry Summary ==========" << std::endl;
        std::cout << "Successful GICP: " << successful_gicp << std::endl;
        std::cout << "Failed GICP: " << failed_gicp << std::endl;
        std::cout << "Total BetweenFactors added: " << successful_gicp << std::endl;
        std::cout << "==========================================\n" << std::endl;
    }
    
    addLoopToGraph(graph, initial, key_frame_timestamps , pointclouds_path, loop_config);

    // exit (0);
 
    std::cout << "Optimizing the factor graph" << std::endl;

    // Load LM optimizer parameters from config
    gtsam::LevenbergMarquardtParams params_lm;
    params_lm.setVerbosity(config["optimization"]["lm"]["verbosity"].as<std::string>("ERROR"));
    params_lm.setMaxIterations(config["optimization"]["lm"]["max_iterations"].as<int>(100));
    params_lm.setLinearSolverType(config["optimization"]["lm"]["linear_solver_type"].as<std::string>("MULTIFRONTAL_QR"));
    params_lm.lambdaInitial = config["optimization"]["lm"]["lambda_initial"].as<double>(0.1);
    params_lm.lambdaFactor = config["optimization"]["lm"]["lambda_factor"].as<double>(3.0);
    params_lm.lambdaUpperBound = config["optimization"]["lm"]["lambda_upper_bound"].as<double>(1e8);
    params_lm.lambdaLowerBound = config["optimization"]["lm"]["lambda_lower_bound"].as<double>(1e-9);
    params_lm.relativeErrorTol = config["optimization"]["lm"]["relative_error_tol"].as<double>(1e-3);
    params_lm.absoluteErrorTol = config["optimization"]["lm"]["absolute_error_tol"].as<double>(1e-3);
    
    std::cout << "\n========== Using LevenbergMarquardtOptimizer ==========" << std::endl;
    std::cout << "  Max iterations: " << params_lm.maxIterations << std::endl;
    std::cout << "  Lambda initial: " << params_lm.lambdaInitial << " (damping factor)" << std::endl;
    std::cout << "  Convergence tol: rel=" << params_lm.relativeErrorTol 
              << ", abs=" << params_lm.absoluteErrorTol << std::endl;
    std::cout << "  ✓ Robust to weight conflicts and bad loop closures" << std::endl;
    std::cout << "========================================================\n" << std::endl;

    gtsam::LevenbergMarquardtOptimizer optimizer_LM(graph, initial, params_lm);
    Values result = optimizer_LM.optimize();

    std::cout << "Optimization complete" << std::endl;

    // write optimized graph to g2o file
    if (!output_g2o_file.empty())
    {
        try
        {
            writeG2o(graph, result, output_g2o_file);
            std::cout << "Wrote optimized graph to: " << output_g2o_file << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to write g2o file: " << e.what() << std::endl;
        }
    }

    auto opt_enu_poses = saveTUMTrajectory(result, opt_tum_file, key_frame_timestamps);

    // 基于 ENU 原点经纬高，利用 GeographicLib 将优化后的 ENU 轨迹反算为经纬度再投影到 UTM
    auto utm_opt_pose = to_utm_pose(opt_enu_poses, temp_file_dir);

    cloud_ptr global_map(new pcl::PointCloud<pointtype>());
    // for (const auto pose : utm_opt_pose)
    for (int i = 0; i < utm_opt_pose.size(); ++i)
    {
        const auto &pose = utm_opt_pose[i];
        const auto pts = loadPCDForIdTimestamp(pointclouds_path, i, pose.timestamp);
        if (!pts || pts->empty())
        {
            ROS_WARN("Skip frame %d: empty/null point cloud", i);
            continue;
        }
        if( i % 500 == 0 )
        {
            std::cerr << i << "th , pts, now map size: " << pts->size() <<  std::endl;
            std::cerr << i << "th , pts_to_world, now map size: " << global_map->size() <<  std::endl;
        }
        cloud_ptr pts_world(new pcl::PointCloud<pointtype>());
        pts_to_world(pts, pts_world, pose);
        *global_map += *pts_world;
    }
    const auto tile_output_dir = temp_file_dir + "/grid_map/";
    save_map_grid(tile_output_dir, global_map, utm_opt_pose);

    return 0;
}
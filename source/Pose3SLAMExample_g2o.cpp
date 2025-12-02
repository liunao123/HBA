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

#include <gtsam/geometry/Pose2.h>

#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// PCL for GICP
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
// Use nano_gicp instead of PCL GICP
#include <nano_gicp/nano_gicp.h>

using namespace std;
using namespace gtsam;

typedef pcl::PointXYZI pointtype;
typedef std::shared_ptr<pcl::PointCloud<pointtype>> cloud_ptr;

struct pose
{
    pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
         Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)) : q(_q), t(_t) {}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
};

std::map<std::string, cloud_ptr> pointCloudCache;
std::mutex cacheMutex;

void applyAdvancedVoxelFilter(const pcl::PointCloud<pointtype> &input_cloud,
                                            pcl::PointCloud<pointtype> &output_cloud)
{
    output_cloud.clear();
    output_cloud.reserve(input_cloud.size());

    // 针对拐弯重影问题的滤波参数优化
    const float min_distance = 2.50;  // 减小最小距离，保留近距离精确点
    const float max_distance = 80.0; // 减小最大距离，避免远距离噪声
    const float min_z = -2.30;         // 收紧高度范围
    const float max_z = 20.0;         // 收紧高度范围

    // 首先进行距离和高度滤波
    for (const auto &p : input_cloud.points)
    {
        Eigen::Vector3d vec(p.x, p.y, p.z);
        const float distance = vec.norm();
        if (distance < min_distance || distance > max_distance)
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
    
    std::cout << "Before voxel filter: " << output_cloud.size() << " points" << std::endl;
    // 应用体素网格滤波
    pcl::VoxelGrid<pointtype> voxel_grid;
    const float voxel_size = 0.15f;
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size); // 5cm分辨率，适合高精度配准
    voxel_grid.setInputCloud(output_cloud.makeShared());
    voxel_grid.filter(output_cloud);
    std::cout << "After voxel filter: " << output_cloud.size() << " points" << std::endl;
}

cloud_ptr getCachedPointCloud(std::string pcd_filename)
{
    // 检查缓存中是否已经有这个文件
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        auto it = pointCloudCache.find(pcd_filename);
        if (it != pointCloudCache.end() && it->second)
        {
            std::cout << "Using cached point cloud: " << pcd_filename 
                      << " (" << it->second->size() << " points)" << std::endl;
            return it->second;
        }
    }

    // 缓存中没有，需要加载
    cloud_ptr cloud(new pcl::PointCloud<pointtype>());

    try
    {
    
    if (pcl::io::loadPCDFile<pointtype>(pcd_filename, *cloud) == -1)
    {
        ROS_ERROR("Failed to load point cloud: %s", pcd_filename.c_str());
        // return nullptr;
    }  
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    std::cout << "Loaded point cloud: " << pcd_filename 
              << " (" << cloud->size() << " points)" << std::endl;
    
    
    cloud_ptr cloud_ds(new pcl::PointCloud<pointtype>());
    applyAdvancedVoxelFilter(*cloud, *cloud_ds);

    // 存入缓存
    {
        std::lock_guard<std::mutex> lock(cacheMutex);
        pointCloudCache[pcd_filename] = cloud_ds;
    }

    return cloud_ds;
}

// Function to read timestamps from TUM file
std::vector<pose> readTumPose(const string &tumFile)
{
    std::vector<pose> pose_vec;
    ifstream file(tumFile);
    if (!file.is_open())
    {
        cerr << "Warning: Cannot open TUM file for timestamps: " << tumFile << endl;
        return pose_vec;
    }
    cout << "Reading timestamps from TUM file: " << tumFile << endl;
    string line;
    while (getline(file, line))
    {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;

        if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)
        {
            Eigen::Quaterniond q(qw, qx, qy, qz);
            Eigen::Vector3d t(tx, ty, tz);
            pose_vec.push_back(pose(q, t));
            // std::cout << "   timestamp:   " << std::to_string(  timestamp ) << std::endl;
        }
    }

    file.close();
    cout << "Read " << pose_vec.size() << " pose from TUM file" << endl;
    
    // Apply offset: subtract the first pose XYZ from all poses
    if (!pose_vec.empty())
    {
        Eigen::Vector3d t_first = pose_vec[0].t;
        
        cout << "Applying XYZ offset - First pose position: t=[" << t_first.transpose() << "]" << endl;
        
        for (size_t i = 0; i < pose_vec.size(); ++i)
        {
            // Only subtract the translation, keep rotation unchanged
            pose_vec[i].t = pose_vec[i].t - t_first;
        }
        
        cout << "XYZ offset applied. First pose position is now at origin." << endl;
    }
    
    return pose_vec;
}

// Function to save trajectory in TUM format with optional timestamps
bool saveTUMTrajectory(const Values &values, const string &filename, const map<Key, double> &timestamps = map<Key, double>())
{
    cout << "Saving trajectory in TUM format to: " << filename << endl;

    ofstream file(filename);
    if (!file.is_open())
    {
        cerr << "Cannot open file: " << filename << endl;
        return false;
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
    return true;
}

// Read timestamps from TUM file into a map<Key,double> where key is line index (0-based)
map<Key, double> readTUMTimestamps(const string &tumFile)
{
    map<Key, double> timestamps;
    ifstream file(tumFile);
    if (!file.is_open())
    {
        cerr << "Warning: Cannot open TUM file for timestamps: " << tumFile << endl;
        return timestamps;
    }
    string line;
    Key currentKey = 0;
    while (getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)
        {
            timestamps[currentKey] = timestamp;
            ++currentKey;
        }
    }
    file.close();
    return timestamps;
}

// Build a simple pose graph and initial values from a sequence of TUM poses
// Each pose in tum_pose becomes a node with key = index (0-based). BetweenFactors
// are added between consecutive poses using the relative transform computed from the poses.
void buildGraphFromTum(const std::vector<pose> &tum_pose,
                       const std::vector<pose> &gicp_pose,
                       NonlinearFactorGraph &graph,
                       Values &initial,
                       double trans_sigma = 0.03,
                       double rot_sigma = 0.001)
{
    size_t n = tum_pose.size();
    if (n == 0)
        return;

    // noise model for odometry (between factors)
    double trans_var = trans_sigma * trans_sigma;
    double rot_var = rot_sigma * rot_sigma;
    gtsam::Vector odomVars = (gtsam::Vector(6) << rot_var, rot_var, rot_var, trans_var, trans_var, trans_var).finished();
    auto odomNoise = noiseModel::Diagonal::Variances(odomVars);

    // prior on first pose (very small to fix gauge)
    gtsam::Vector priorVars = (gtsam::Vector(6) << 1e-8, 1e-8, 1e-8, 1e-4, 1e-4, 1e-4).finished();
    auto priorNoise = noiseModel::Diagonal::Variances(priorVars);

    // create nodes and between factors
    for (size_t i = 0; i < n; ++i)
    {
        const pose &ps = tum_pose[i];
        // tum file provides qx,qy,qz,qw in readTumPose => stored as (qw,qx,qy,qz) in quaternion
        gtsam::Quaternion q(ps.q.w(), ps.q.x(), ps.q.y(), ps.q.z());
        gtsam::Rot3 R = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
        gtsam::Point3 t(ps.t.x(), ps.t.y(), ps.t.z());
        gtsam::Pose3 curr(R, t);

        Key key = static_cast<Key>(i);
        initial.insert(key, curr);

        if (i == 0)
        {
            // add a strong prior to fix the first pose
            graph.add(PriorFactor<Pose3>(key, curr, priorNoise));
        }
        else
        {
            // add between factor between i-1 and i
            const pose &ps_prev = gicp_pose[i - 1];
            gtsam::Quaternion qprev(ps_prev.q.w(), ps_prev.q.x(), ps_prev.q.y(), ps_prev.q.z());
            gtsam::Rot3 Rprev = gtsam::Rot3::Quaternion(qprev.w(), qprev.x(), qprev.y(), qprev.z());
            gtsam::Point3 tprev(ps_prev.t.x(), ps_prev.t.y(), ps_prev.t.z());
            gtsam::Pose3 prev(Rprev, tprev);


            const pose &gicp_ps = gicp_pose[i];
            gtsam::Quaternion gicp_q(gicp_ps.q.w(), gicp_ps.q.x(), gicp_ps.q.y(), gicp_ps.q.z());
            gtsam::Rot3 R1 = gtsam::Rot3::Quaternion(gicp_q.w(), gicp_q.x(), gicp_q.y(), gicp_q.z());
            gtsam::Point3 t1(gicp_ps.t.x(), gicp_ps.t.y(), gicp_ps.t.z());
            gtsam::Pose3 curr_1(R1, t1);

            gtsam::Pose3 meas = prev.between(curr_1);
            graph.add(BetweenFactor<Pose3>(static_cast<Key>(i - 1), key, meas, odomNoise));
        }
    }
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
    ss << std::fixed << std::setprecision(3) << timestamp << ".pcd";
    // ss << id << "_" << std::fixed << std::setprecision(3) << timestamp << ".pcd";
    std::string filename = ss.str();
    // std::cout << "filename " << filename << " done " << std::endl;

    cloud_ptr cloud(new pcl::PointCloud<pointtype>());
    // if (pcl::io::loadPCDFile(filename, *cloud) == -1)
    // {
    //     // ROS_WARN("Failed to load PCD file: %s", filename.c_str());
    //     return nullptr;
    // }
    cloud = getCachedPointCloud(filename);
    std::cout << "281 :::cloud->size() :  " << cloud->size() << " done " << std::endl;

    return cloud;
}



// Run GICP between two clouds, return transform mapping source->target and fitness score
bool runGICPGetRelative(const cloud_ptr &target,
                        const cloud_ptr &source,
                        const Eigen::Matrix4f &init_guess,
                        Eigen::Matrix4f &out_T,
                        double &out_fitness,
                        int max_iter = 50,
                        double voxel_size = 0.2)
{
    // basic null checks
    if (!target || !source)
    {
        ROS_WARN("runGICPGetRelative: null cloud pointer");
        return false;
    }

    if (target->empty() || source->empty())
    {
        ROS_WARN("runGICPGetRelative: empty input clouds (target=%zu, source=%zu)", target->size(), source->size());
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

    // downsample
    cloud_ptr target_ds(new pcl::PointCloud<pointtype>());
    cloud_ptr source_ds(new pcl::PointCloud<pointtype>());

    pcl::copyPointCloud(*target, *target_ds);
    pcl::copyPointCloud(*source, *source_ds);

    // ROS_INFO("runGICP: target_ds=%zu source_ds=%zu (voxel=%.3f)", target->size(), source->size(), voxel_size);
 

    ROS_INFO("runGICP: target_ds=%zu source_ds=%zu (voxel=%.3f)", target_ds->size(), source_ds->size(), voxel_size);

    if (target_ds->empty() || source_ds->empty())
    {
        ROS_WARN("runGICPGetRelative: downsampled clouds are empty");
        return false;
    }

    nano_gicp::NanoGICP<pointtype, pointtype> gicp;
    gicp.setCorrespondenceRandomness(128);
    gicp.setMaxCorrespondenceDistance(0.50);
    gicp.setMaximumIterations(128);
    gicp.setTransformationEpsilon(1e-3);
    gicp.setRotationEpsilon(1e-3);
    gicp.setInitialLambdaFactor(1e-9);
    gicp.setRegularizationMethod(nano_gicp::RegularizationMethod::PLANE);
    gicp.setInputSource(source_ds);
    gicp.setInputTarget(target_ds);
    gicp.calculateSourceCovariances();
    gicp.calculateTargetCovariances();

    pcl::PointCloud<pointtype> Final;
    // gicp.align(Final);

    try
    {
        gicp.align(Final, init_guess);
        // gicp.align(Final);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("GICP align threw exception: %s", e.what());
        return false;
    }

    if (!gicp.hasConverged())
    {
        ROS_INFO("GICP did not converge (fitness=%f)", gicp.getFitnessScore());
        return false;
    }

    Eigen::Matrix4f final_tf = gicp.getFinalTransformation();
    // final_tf maps source_init -> target_ds; source_init = init_guess * source_ds
    // Therefore, mapping source -> target is final_tf * init_guess
    out_T = final_tf; //* init_guess;
    out_fitness = gicp.getFitnessScore();

    static int cnt = 0;
    std::cout << "doooooooooooooo icp align ee" << std::endl;
    std::string savePCDDirectory = "./";
    mkdir((savePCDDirectory + "/loop_closure/").c_str(), 0777);
    pcl::io::savePCDFileBinary(savePCDDirectory + "/loop_closure/" + std::to_string(cnt) + "unused_result.pcd", Final);
    pcl::io::savePCDFileBinary(savePCDDirectory + "/loop_closure/" + std::to_string(cnt) + "prevKeyframeCloud.pcd", *target_ds);
    pcl::io::savePCDFileBinary(savePCDDirectory + "/loop_closure/" + std::to_string(cnt) + "cureKeyframeCloud.pcd_" + std::to_string(out_fitness), *source_ds);
    cnt++;

    ROS_INFO("GICP converged fitness=%f", out_fitness);
    return true;
}

// Add loop closures between poses based on time and spatial thresholds using GICP on PCDs
void addLoopToGraph(NonlinearFactorGraph &graph,
                    const Values &initial,
                    const map<Key, double> &timestamps,
                    const std::string &pcd_dir,
                    double time_thresh = 20.0,
                    double spatial_thresh = 10.0,
                    double fitness_thresh = 1.0,
                    double voxel_size = 0.2,
                    int max_iter = 50,
                    double variance_scale = 0.10)
{
    // collect keys in initial
    std::vector<Key> keys;
    for (const auto &kv : initial)
        keys.push_back(kv.key);
    std::sort(keys.begin(), keys.end());

    for (size_t i = 0; i < keys.size(); i = i + 2)
    // for (size_t i = 0; i < 20; i = i + 2)
    {
        // for (size_t i = 0; i < 100; ++i) {
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

        for (size_t j = i + 3; j < keys.size(); j = j + 2)
        { // skip adjacent (i+1)
            Key kj = keys[j];
            if (!initial.exists<Pose3>(kj))
                continue;
            Pose3 pj = initial.at<Pose3>(kj);
            double tj = 0;
            auto it_tj = timestamps.find(kj);
            if (it_tj != timestamps.end())
                tj = it_tj->second;

            if (std::fabs(tj - ti) < time_thresh)
                continue;
            if (std::fabs(kj - ki) < 20)
                continue;

            // spatial distance
            Eigen::Vector3d di(pi.translation().x() - pj.translation().x(),
                               pi.translation().y() - pj.translation().y(),
                               pi.translation().z() - pj.translation().z());
            double dist = di.norm();
            if (dist > spatial_thresh)
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
            std::cout << "Running GICP between " << ki << " and " << kj << std::endl;
            bool ok = runGICPGetRelative(cloud_i, cloud_j, init_guess, Tij, fitness, max_iter, voxel_size);
            std::cout << "Running GICP between " << ki << " and " << kj << std::endl;
            if (!ok)
                continue;
            std::cout << "ooooooooooookkkkkkkkk Running GICP between " << ki << " and " << kj << std::endl;

            // if (fitness > fitness_thresh) {
            //     std::cout << "Rejected loop " << ki << "-" << kj << " fitness=" << fitness << std::endl;
            //     continue;
            // }

            Pose3 meas = eigenMatrix4fToPose3(Tij);
            double var_trans = std::max(1e-2, fitness * variance_scale);
            double var_rot = std::max(1e-3, fitness * variance_scale * 0.1);
            gtsam::Vector variances = (gtsam::Vector(6) << var_rot, var_rot, var_rot, var_trans, var_trans, var_trans).finished();
            auto noise = noiseModel::Diagonal::Variances(variances);
            graph.add(BetweenFactor<Pose3>(ki, kj, meas, noise));
            std::cout << "Added loop constraint between " << ki << " and " << kj << " dist=" << dist << " time_diff=" << std::fabs(tj - ti) << " fitness=" << fitness << std::endl;
            ROS_ERROR("Added loop constraint between %d and %d : %f", ki, kj, fitness);
            // i = i + 2; // skip some to speed up
            // break;
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS for parameter reading
    ros::init(argc, argv, "pose3_slam_g2o");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 获取参数
    std::string output_g2o_file, output_tum_file, gnss_tum_file, gicp_tum_file;
    std::string optimization_method;
    // std::string work_dir = "/home/xf/Desktop/catkin_ws/data/28_3/";
    std::string work_dir = "/mnt/nvme0n1p2/data/nongan_m2_1028/";

    pnh.param<std::string>("output_g2o_file", output_g2o_file, work_dir + "/graph_opt.g2o");
    pnh.param<std::string>("gnss_tum_file", gnss_tum_file, work_dir + "/gnss_lidar_pose_trajectory.tum");
    pnh.param<std::string>("gicp_tum_file", gicp_tum_file, work_dir + "/gicp_trajectory.tum");
    pnh.param<std::string>("output_tum_file", output_tum_file, work_dir + "/graph_opt_trajectory.tum");
    pnh.param<std::string>("optimization_method", optimization_method, "ISAM2"); // "LM" or "ISAM2"

    auto gnss_pose = readTumPose(gnss_tum_file);
    auto gicp_pose = readTumPose(gicp_tum_file);

    // Build graph and initial estimate from TUM poses
    NonlinearFactorGraph graph;
    Values initial;

    buildGraphFromTum(gnss_pose, gicp_pose, graph, initial);
    std::cout << "buildGraphFromTum done " << std::endl;

    // read timestamps for mapping keys -> timestamps
    map<Key, double> timestamps = readTUMTimestamps(gnss_tum_file);
    std::cout << "buildGraphFromTum done " << std::endl;

    // loop closure / ICP parameters (can be set via ROS params)
    std::string pcd_dir;
    double time_thresh = 10.0;
    double spatial_thresh = 10.0;
    double fitness_thresh = 1.0;
    double voxel_size = 0.1;
    int icp_max_iter = 50;
    double variance_scale = 10.0;

    pnh.param<std::string>("pcd_dir", pcd_dir, work_dir + "/pcd/");
    pnh.param<double>("icp_time_thresh", time_thresh, time_thresh);
    pnh.param<double>("icp_spatial_thresh", spatial_thresh, spatial_thresh);
    pnh.param<double>("icp_fitness_thresh", fitness_thresh, fitness_thresh);
    pnh.param<double>("icp_voxel_size", voxel_size, voxel_size);
    pnh.param<int>("icp_max_iter", icp_max_iter, icp_max_iter);
    pnh.param<double>("icp_variance_scale", variance_scale, variance_scale);

    // Add loop closures based on GICP between PCDs
    addLoopToGraph(graph, initial, timestamps, pcd_dir, time_thresh, spatial_thresh, fitness_thresh, voxel_size, icp_max_iter, variance_scale);

    std::cout << "Optimizing the factor graph" << std::endl;

    gtsam::LevenbergMarquardtParams params_lm;        // 构建LM算法参数类(相当于g2o options)
    params_lm.setVerbosity("ERROR");                  // 设置输出信息
    params_lm.setMaxIterations(50);                   // 最大迭代次数
    params_lm.setLinearSolverType("MULTIFRONTAL_QR"); // 分解算法
    params_lm.lambdaInitial = 1e-3;                   // 初始阻尼
    params_lm.lambdaFactor = 2.0;                     // λ 扩大倍数
    params_lm.lambdaUpperBound = 1e6;                 // λ 上限
    params_lm.lambdaLowerBound = 1e-7;                // λ 下限
    params_lm.maxIterations = 50;
    params_lm.relativeErrorTol = 1e-5;
    params_lm.absoluteErrorTol = 1e-7;

    gtsam::LevenbergMarquardtOptimizer optimizer_LM(graph, initial, params_lm); // 构建下降算法(图,初值,参数)

    Values result = optimizer_LM.optimize(); // 开始优化

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

    if (!output_tum_file.empty())
    {
        if (saveTUMTrajectory(result, output_tum_file, timestamps))
        {
            std::cout << "Wrote optimized TUM trajectory to: " << output_tum_file << std::endl;
        }
        else
        {
            std::cerr << "Failed to save TUM trajectory" << std::endl;
        }
    }

    GaussNewtonParams params;
    params.setVerbosity("TERMINATION"); // show info about stopping conditions
    GaussNewtonOptimizer optimizer(graph, initial, params);
    result = optimizer.optimize();
    // write optimized graph to g2o file
    {
        try
        {
            writeG2o(graph, result, "./graph_opt_loop_gn.g2o");
            std::cout << "Wrote optimized graph to: " << output_g2o_file << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to write g2o file: " << e.what() << std::endl;
        }
    }

    {
        if (saveTUMTrajectory(result, "./tum_gn.tum", timestamps))
        {
            std::cout << "Wrote optimized TUM trajectory to: " << output_tum_file << std::endl;
        }
        else
        {
            std::cerr << "Failed to save TUM trajectory" << std::endl;
        }
    }

    return 0;
}
#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// for taojian ot 128
// struct PandarPointXYZIRT {
//     PCL_ADD_POINT4D;
//     // uint8_t intensity;
//     float intensity;
//     double timestamp;
//     uint16_t ring;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT( PandarPointXYZIRT,
//         (float, x, x)
//         (float, y, y)
//         (float, z, z)
//         // (uint8_t, intensity, intensity)
//         (float, intensity, intensity)
//         (double, timestamp, timestamp)
//         (uint16_t, ring, ring)
// )

// for id4
struct PandarPointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    // float intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT( PandarPointXYZIRT,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (uint8_t, intensity, intensity)
        // (float, intensity, intensity)
        (double, timestamp, timestamp)
        (uint16_t, ring, ring)
)


// Use TumPose for both GNSS and LIO-style poses. Fields:
//   - q: orientation
//   - t: position (UTM meters)
//   - timestamp: seconds
struct TumPose
{
    TumPose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
         Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0), double _ts = 0.0) : q(_q), t(_t), timestamp(_ts) {}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    double timestamp;
};

// Voxel filter parameters
extern float g_min_distance;
extern float g_max_distance;
extern float g_min_z;
extern float g_max_z;
extern float g_voxel_size;

// GICP validation parameters
extern float g_max_trans_diff;
extern float g_max_rot_diff;

// GICP parameters
extern int g_gicp_correspondence_randomness;
extern float g_gicp_max_correspondence_distance;
extern int g_gicp_max_iterations;
extern float g_gicp_transformation_epsilon;
extern float g_gicp_rotation_epsilon;
extern float g_gicp_initial_lambda_factor;

// Structure to hold noise parameters from config
struct NoiseConfig {
    double odom_rot_std;
    double odom_trans_std_xy;
    double odom_trans_std_z;
    double gnss_position_std_xy;
    double gnss_position_std_z;
    double gnss_prior_multiplier;
    int gnss_prior_interval;
};


struct LoopConfig {
    double time_thresh;
    double spatial_thresh;
    double fitness_thresh;
    double max_fitness_reject;
    double voxel_size;
    int max_iter;
    double variance_scale;
    double var_trans;
    double var_rot;
    int start_index;
    int end_index;
    int step;
    int min_key_diff;
};

// GNSS odometry data structure from YAML files
// 结构体用于队列缓存GNSS和LiDAR数据
struct GnssOdomData
{
    Eigen::Vector3d position;        // [x, y, z] in ENU
    double timestamp;
    Eigen::Vector3d position_lla;    // [lat, lon, alt]
    Eigen::Matrix3d pose;            // 3x3 rotation matrix
    Eigen::Vector3d enu_velocity;    // velocity in ENU frame
    double heading;                  // heading angle
    double speed;                    // speed magnitude
};

// Read a single GNSS odom YAML file
inline GnssOdomData readGnssOdomYaml(const std::string& yaml_file)
{
    GnssOdomData data;
    
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        // Read position
        if (config["position"] && config["position"].size() == 3) {
            data.position = Eigen::Vector3d(
                config["position"][0].as<double>(),
                config["position"][1].as<double>(),
                config["position"][2].as<double>()
            );
        }
        
        // Read timestamp
        if (config["timestamp"]) {
            data.timestamp = config["timestamp"].as<double>();
        }
        
        // Read position_lla
        if (config["position_lla"] && config["position_lla"].size() == 3) {
            data.position_lla = Eigen::Vector3d(
                config["position_lla"][0].as<double>(),
                config["position_lla"][1].as<double>(),
                config["position_lla"][2].as<double>()
            );
        }
        
        // Read pose (3x3 rotation matrix stored as 9 values)
        if (config["pose"] && config["pose"].size() == 9) {
            data.pose << 
                config["pose"][0].as<double>(), config["pose"][1].as<double>(), config["pose"][2].as<double>(),
                config["pose"][3].as<double>(), config["pose"][4].as<double>(), config["pose"][5].as<double>(),
                config["pose"][6].as<double>(), config["pose"][7].as<double>(), config["pose"][8].as<double>();
        }
        
        // Read enu_velocity
        if (config["enu_velocity"] && config["enu_velocity"].size() == 3) {
            data.enu_velocity = Eigen::Vector3d(
                config["enu_velocity"][0].as<double>(),
                config["enu_velocity"][1].as<double>(),
                config["enu_velocity"][2].as<double>()
            );
        }
        
        // Read heading
        if (config["heading"]) {
            data.heading = config["heading"].as<double>();
        }
        
        // Read speed
        if (config["speed"]) {
            data.speed = config["speed"].as<double>();
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error reading GNSS odom YAML file " << yaml_file << ": " << e.what() << std::endl;
    }
    
    return data;
}

// Read all GNSS odom YAML files from a list of file paths
inline std::vector<GnssOdomData> readAllGnssOdomYamls(const std::vector<std::string>& yaml_files)
{
    std::vector<GnssOdomData> all_data;
    all_data.reserve(yaml_files.size());
    
    std::cout << "Reading " << yaml_files.size() << " GNSS odom YAML files..." << std::endl;
    
    for (const auto& yaml_file : yaml_files) {
        GnssOdomData data = readGnssOdomYaml(yaml_file);
        all_data.push_back(data);
    }
    
    std::cout << "Successfully read " << all_data.size() << " GNSS odom data entries" << std::endl;
    
    return all_data;
}

// LiDAR-GNSS extrinsic calibration structure
struct LidarGnssExtrinsic {
    Eigen::Quaterniond q;  // Rotation from GNSS to LiDAR
    Eigen::Vector3d t;     // Translation from GNSS to LiDAR
    
    // Get transformation matrix
    Eigen::Matrix4d getTransformMatrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        T.block<3,1>(0,3) = t;
        return T;
    }
};

// Convert GNSS odometry data to LiDAR poses in world frame using extrinsic calibration
inline std::vector<TumPose> GetLidarPoseOnWorld(
    const std::vector<GnssOdomData>& gnss_odom_data,
    const LidarGnssExtrinsic& extrinsic)
{
    std::vector<TumPose> lidar_poses;
    lidar_poses.reserve(gnss_odom_data.size());
    
    Eigen::Matrix4d T_gnss_lidar = extrinsic.getTransformMatrix();
    
    std::cout << "\n========== Converting GNSS poses to LiDAR poses ==========" << std::endl;
    std::cout << "Extrinsic transformation T_gnss_lidar:" << std::endl;
    std::cout << T_gnss_lidar << std::endl;
    
    for (size_t i = 0; i < gnss_odom_data.size(); ++i) {
        const auto& gnss_data = gnss_odom_data[i];
        
        // Construct GNSS pose matrix
        Eigen::Matrix4d T_world_gnss = Eigen::Matrix4d::Identity();
        T_world_gnss.block<3,3>(0,0) = gnss_data.pose;
        T_world_gnss.block<3,1>(0,3) = gnss_data.position;
        
        // Apply extrinsic: T_world_lidar = T_world_gnss * T_gnss_lidar
        // Note: T_gnss_lidar is the transformation from GNSS to LiDAR
        Eigen::Matrix4d T_world_lidar = T_world_gnss * T_gnss_lidar;
        
        // Extract rotation and translation
        Eigen::Matrix3d R_lidar = T_world_lidar.block<3,3>(0,0);
        Eigen::Vector3d t_lidar = T_world_lidar.block<3,1>(0,3);
        Eigen::Quaterniond q_lidar(R_lidar);
        
        // Create TumPose
        TumPose lidar_pose;
        lidar_pose.timestamp = gnss_data.timestamp;
        lidar_pose.q = q_lidar;
        lidar_pose.t = t_lidar;
        
        lidar_poses.push_back(lidar_pose);
    }
    
    std::cout << "Converted " << lidar_poses.size() << " GNSS poses to LiDAR frame" << std::endl;
    if (!lidar_poses.empty()) {
        std::cout << "First LiDAR pose - timestamp: " << lidar_poses[0].timestamp
                  << ", position: [" << lidar_poses[0].t.transpose() << "]" << std::endl;
        std::cout << "Last LiDAR pose - timestamp: " << lidar_poses.back().timestamp
                  << ", position: [" << lidar_poses.back().t.transpose() << "]" << std::endl;
    }
    std::cout << "==========================================================\n" << std::endl;
    
    return lidar_poses;
}

// Function to load configuration from YAML file
// 新版参数顺序，适配Pose3SLAMExample_g2o.cpp
inline void loadConfigFromYAML(const std::string& config_file,
                               std::string& work_dir,
                               std::string& optimization_method,
                               NoiseConfig& noise_config,
                               LoopConfig& loop_config,
                               bool& save_loop_g2o,
                               bool& load_loop_g2o,
                               LidarGnssExtrinsic& extrinsic,
                               YAML::Node& config)
{
    try {
        config = YAML::LoadFile(config_file);
        std::cout << "Loaded configuration from: " << config_file << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load config file: " << e.what() << std::endl;
        std::cerr << "Using default parameters" << std::endl;
        config = YAML::Node();
    }

    // Load voxel filter parameters
    g_min_distance = config["voxel_filter"]["min_distance"].as<float>(2.50);
    g_max_distance = config["voxel_filter"]["max_distance"].as<float>(80.0);
    g_min_z = config["voxel_filter"]["min_z"].as<float>(-2.30);
    g_max_z = config["voxel_filter"]["max_z"].as<float>(20.0);
    g_voxel_size = config["voxel_filter"]["voxel_size"].as<float>(0.15);

    // Load GICP parameters
    g_gicp_correspondence_randomness = config["gicp"]["correspondence_randomness"].as<int>(128);
    g_gicp_max_correspondence_distance = config["gicp"]["max_correspondence_distance"].as<float>(0.50);
    g_gicp_max_iterations = config["gicp"]["max_iterations"].as<int>(128);
    g_gicp_transformation_epsilon = config["gicp"]["transformation_epsilon"].as<float>(0.001);
    g_gicp_rotation_epsilon = config["gicp"]["rotation_epsilon"].as<float>(0.001);
    g_gicp_initial_lambda_factor = config["gicp"]["initial_lambda_factor"].as<float>(1e-9);

    // Load GICP validation parameters
    g_max_trans_diff = config["gicp_validation"]["max_trans_diff"].as<float>(2.0);
    g_max_rot_diff = config["gicp_validation"]["max_rot_diff"].as<float>(0.5);

    // Load file paths
    work_dir = config["paths"]["work_dir"].as<std::string>("/mnt/nvme0n1p2/data/nongan_m2_1028/");

    // Load optimization parameters
    optimization_method = config["optimization"]["method"].as<std::string>("ISAM2");

    // Load noise model configuration
    noise_config.odom_rot_std = config["noise"]["odometry"]["rotation_std"].as<double>(0.003);
    noise_config.odom_trans_std_xy = config["noise"]["odometry"]["translation_std_xy"].as<double>(0.5);
    noise_config.odom_trans_std_z = config["noise"]["odometry"]["translation_std_z"].as<double>(0.5);
    noise_config.gnss_position_std_xy = config["noise"]["gnss"]["position_std_xy"].as<double>(0.15);
    noise_config.gnss_position_std_z = config["noise"]["gnss"]["position_std_z"].as<double>(0.25);
    noise_config.gnss_prior_multiplier = config["noise"]["gnss"]["gnss_prior_multiplier"].as<double>(100);
    noise_config.gnss_prior_interval = config["noise"]["gnss"]["gnss_prior_interval"].as<int>(500);

    // Load loop closure configuration
    loop_config.time_thresh = config["loop_closure"]["time_thresh"].as<double>(10.0);
    loop_config.spatial_thresh = config["loop_closure"]["spatial_thresh"].as<double>(10.0);
    loop_config.fitness_thresh = config["loop_closure"]["fitness_thresh"].as<double>(1.0);
    loop_config.max_fitness_reject = config["loop_closure"]["max_fitness_reject"].as<double>(50.0);
    loop_config.voxel_size = config["loop_closure"]["voxel_size"].as<double>(0.1);
    loop_config.max_iter = config["loop_closure"]["max_iter"].as<int>(50);
    loop_config.variance_scale = config["loop_closure"]["variance_scale"].as<double>(10.0);
    loop_config.var_trans = config["loop_closure"]["noise"]["var_trans"].as<double>(0.010);
    loop_config.var_rot = config["loop_closure"]["noise"]["var_rot"].as<double>(0.005);
    loop_config.start_index = config["loop_closure"]["search"]["start_index"].as<int>(1200);
    loop_config.end_index = config["loop_closure"]["search"]["end_index"].as<int>(2000);
    loop_config.step = config["loop_closure"]["search"]["step"].as<int>(5);
    loop_config.min_key_diff = config["loop_closure"]["search"]["min_key_diff"].as<int>(20);

    save_loop_g2o = config["loop_closure"]["save_loop_g2o"].as<bool>(true);
    load_loop_g2o = config["loop_closure"]["load_loop_g2o"].as<bool>(false);

    // Load LiDAR-GNSS extrinsic calibration
    extrinsic.q.w() = config["lidar_gnss_extrinsic"]["quaternion"]["w"].as<double>(1.0);
    extrinsic.q.x() = config["lidar_gnss_extrinsic"]["quaternion"]["x"].as<double>(0.0);
    extrinsic.q.y() = config["lidar_gnss_extrinsic"]["quaternion"]["y"].as<double>(0.0);
    extrinsic.q.z() = config["lidar_gnss_extrinsic"]["quaternion"]["z"].as<double>(0.0);
    extrinsic.t.x() = config["lidar_gnss_extrinsic"]["translation"]["x"].as<double>(0.0);
    extrinsic.t.y() = config["lidar_gnss_extrinsic"]["translation"]["y"].as<double>(0.0);
    extrinsic.t.z() = config["lidar_gnss_extrinsic"]["translation"]["z"].as<double>(0.0);

    std::cout << "\n========== Configuration Summary ==========" << std::endl;
    std::cout << "Work directory: " << work_dir << std::endl;
    std::cout << "Optimization method: " << optimization_method << std::endl;
    std::cout << "Odometry noise - rot: " << noise_config.odom_rot_std << " trans_xy: " << noise_config.odom_trans_std_xy << std::endl;
    std::cout << "GNSS noise - xy: " << noise_config.gnss_position_std_xy << " z: " << noise_config.gnss_position_std_z << std::endl;
    std::cout << "Loop closure - time_thresh: " << loop_config.time_thresh << " spatial: " << loop_config.spatial_thresh << std::endl;
    std::cout << "LiDAR-GNSS extrinsic - q: [" << extrinsic.q.w() << ", " << extrinsic.q.x() << ", " 
              << extrinsic.q.y() << ", " << extrinsic.q.z() << "], t: [" 
              << extrinsic.t.transpose() << "]" << std::endl;
    std::cout << "==========================================\n" << std::endl;
}

#endif // COMMON_HPP

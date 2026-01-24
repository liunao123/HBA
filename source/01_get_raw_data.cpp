#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <chcnav/hcinspvatzcb.h>

// PCL for saving pointclouds
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include "common.hpp"

struct LidarData {
  double timestamp;
};

// Local ENU converter using GeographicLib::LocalCartesian
struct ENUConverter {
  bool initialized = false;
  double lat0 = 0.0, lon0 = 0.0, h0 = 0.0;
  std::unique_ptr<GeographicLib::LocalCartesian> proj;

  void init(double lat_ref, double lon_ref, double h_ref) {
    lat0 = lat_ref; lon0 = lon_ref; h0 = h_ref;
    proj.reset(new GeographicLib::LocalCartesian(lat0, lon0, h0));
    initialized = true;
    ROS_INFO("LocalCartesian ENU initialized at lat=%.8f lon=%.8f h=%.3f", lat0, lon0, h0);
  }

  // convert lat/lon/alt -> ENU (x east, y north, z up)
  void convertToENU(double lat, double lon, double alt, double &x, double &y, double &z) {
    if (!initialized) {
      init(lat, lon, alt);
    }
    proj->Forward(lat, lon, alt, x, y, z);
  }
};


static inline Eigen::Quaterniond rpyDegToQuat(double roll_deg, double pitch_deg, double yaw_deg) {
  const double r = roll_deg * M_PI / 180.0;
  const double p = pitch_deg * M_PI / 180.0;
  const double y = yaw_deg * M_PI / 180.0;
  Eigen::AngleAxisd Rx(r, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(p, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(y, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = Rz * Ry * Rx; // ZYX convention  ok ENU下的姿态顺序
  // Eigen::Quaterniond q = Rx * Ry * Rz; // ZYX convention
  return q.normalized();
}


// 用于记录已保存的odom时间戳
#include <unordered_set>
#include <map>
#include <limits>
static std::unordered_set<double> g_saved_odom_ts;
static std::string g_work_dir = ".";

// 用于记录所有odom和点云的配对关系
static std::map<double, double> g_lidar_to_odom; // lidar_ts -> odom_ts

// Write a single pose YAML file using yaml-cpp
void write_odom_yaml( double timestamp, const Eigen::Vector3d& position,
           double lat, double lon, double alt,
           const Eigen::Matrix3d& R,
           const Eigen::Vector3d& enu_velocity,
           double heading, double speed) {
  // 静态变量保存上一帧的位姿和yaw
  static Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  static double last_yaw = 0.0;
  static bool has_last = false;

  // 计算当前yaw（从R中提取）
  double curr_yaw = std::atan2(R(1,0), R(0,0)) * 180.0 / M_PI; // degree
  // 计算距离和yaw变化
  double dist = has_last ? (position - last_position).norm() : 0.0;
  double dyaw = has_last ? std::fabs(curr_yaw - last_yaw) : 0.0;
  if (dyaw > 180.0) dyaw = 360.0 - dyaw;

  // 只有距离大于1m或yaw变化大于10度才保存
  if (has_last && dist <= 1.0 && dyaw <= 10.0) {
    return;
  }

  // 保存到 work_dir/odoms/
  std::string odom_dir = g_work_dir + "/odoms/";
  struct stat st;
  if (stat(odom_dir.c_str(), &st) != 0) {
    mkdir(odom_dir.c_str(), 0777);
  }
  std::ostringstream yaml_fn;
  static int id = 0;
  yaml_fn << odom_dir << id++ << "_" << std::fixed << std::setprecision(3) << timestamp << ".yaml";
  std::cout << "[YAML] Writing: " << yaml_fn.str() << std::endl;

  YAML::Node node;
  node["position"].push_back(position.x());
  node["position"].push_back(position.y());
  node["position"].push_back(position.z());
  {
    std::ostringstream ts_ss;
    ts_ss << std::fixed << std::setprecision(3) << timestamp;
    node["timestamp"] = ts_ss.str();
  }
  node["position_lla"].push_back(lat);
  node["position_lla"].push_back(lon);
  node["position_lla"].push_back(alt);
  node["pose"].push_back(R(0,0)); node["pose"].push_back(R(0,1)); node["pose"].push_back(R(0,2));
  node["pose"].push_back(R(1,0)); node["pose"].push_back(R(1,1)); node["pose"].push_back(R(1,2));
  node["pose"].push_back(R(2,0)); node["pose"].push_back(R(2,1)); node["pose"].push_back(R(2,2));
  node["enu_velocity"].push_back(enu_velocity.x());
  node["enu_velocity"].push_back(enu_velocity.y());
  node["enu_velocity"].push_back(enu_velocity.z());
  node["heading"] = heading;
  node["speed"] = speed;
  std::ofstream fout(yaml_fn.str());
  fout << node;
  fout.close();

  // 更新上一帧
  last_position = position;
  last_yaw = curr_yaw;
  has_last = true;

  // 记录已保存的odom时间戳
  g_saved_odom_ts.insert(timestamp);
}

// 启动YAML对齐导出线程，返回std::thread对象
std::thread start_yaml_worker(std::deque<GnssOdomData>& gnss_queue, std::deque<LidarData>& lidar_queue,
                              std::mutex& mtx, std::condition_variable& cv, bool& finished) {
  return std::thread([&](){
    while (true) {
      std::unique_lock<std::mutex> lock(mtx);
      cv.wait(lock, [&]{ return !gnss_queue.empty() && !lidar_queue.empty() || finished; });
      if (finished && (gnss_queue.empty() || lidar_queue.empty())) break;
      // 对齐逻辑
      while (!gnss_queue.empty() && !lidar_queue.empty()) {
        double dt = fabs(gnss_queue.front().timestamp - lidar_queue.front().timestamp);
        if (dt < 0.005) {
          // 匹配，导出YAML，lidar时间戳命名
          const auto& g = gnss_queue.front();
          const auto& l = lidar_queue.front();
          // 拆解GnssOdomData结构体，按新定义写入
          write_odom_yaml(
            l.timestamp,
            g.position,
            g.position_lla.x(), g.position_lla.y(), g.position_lla.z(),
            g.pose,
            g.enu_velocity,
            g.heading,
            g.speed
          );
          gnss_queue.pop_front();
          lidar_queue.pop_front();
        } else if (gnss_queue.front().timestamp < lidar_queue.front().timestamp) {
          // GNSS太早，丢弃
          gnss_queue.pop_front();
        } else {
          // LiDAR太早，丢弃
          lidar_queue.pop_front();
        }
      }
    }
  });
}

// 点云去运动畸变，输入原始点云和 enu_poses，返回去畸变后的点云
pcl::PointCloud<pcl::PointXYZI> undistort_pointcloud(
    const pcl::PointCloud<PandarPointXYZIRT> &raw_cloud,
    const std::vector<TumPose> &enu_poses,
    double frame_time)
{
  pcl::PointCloud<pcl::PointXYZI> out_cloud;
  if (enu_poses.size() < 2)
  {
    // 无法插值，直接拷贝
    for (const auto &pt : raw_cloud)
    {
      pcl::PointXYZI p;
      p.x = pt.x; p.y = pt.y; p.z = pt.z; p.intensity = pt.intensity;
      out_cloud.push_back(p);
    }
    return out_cloud;
  }
  // 查找帧起止时间对应的位姿
  double t0 = frame_time, t1 = frame_time;
  for (const auto &pt : raw_cloud)
  {
    if (pt.timestamp < t0) t0 = pt.timestamp;
    if (pt.timestamp > t1) t1 = pt.timestamp;
  }
  // 逐点插值
  for (const auto &pt : raw_cloud)
  {
    double t = pt.timestamp;
    // 找到 t 前后的 enu_pose
    auto it = std::upper_bound(enu_poses.begin(), enu_poses.end(), t,
      [](double v, const TumPose &tp) { return v < tp.timestamp; });
    if (it == enu_poses.begin() || it == enu_poses.end())
    {
      // 超出范围，直接拷贝
      pcl::PointXYZI p;
      p.x = pt.x; p.y = pt.y; p.z = pt.z; p.intensity = pt.intensity;
      out_cloud.push_back(p);
      continue;
    }
    const TumPose &B = *it;
    const TumPose &A = *(it - 1);
    double alpha = (t - A.timestamp) / std::max(1e-6, B.timestamp - A.timestamp);
    Eigen::Quaterniond q = A.q.slerp(alpha, B.q);
    Eigen::Vector3d p0(pt.x, pt.y, pt.z);
    // 以帧时间为参考，反向补偿到帧时刻
    Eigen::Quaterniond q_ref = A.q.slerp((frame_time - A.timestamp) / std::max(1e-6, B.timestamp - A.timestamp), B.q);
    Eigen::Vector3d t_ref = (1.0 - ((frame_time - A.timestamp) / std::max(1e-6, B.timestamp - A.timestamp))) * A.t + ((frame_time - A.timestamp) / std::max(1e-6, B.timestamp - A.timestamp)) * B.t;
    Eigen::Vector3d t_pt = (1.0 - alpha) * A.t + alpha * B.t;
    // 先变换到世界系，再变换回参考帧
    Eigen::Vector3d Pw = q * p0 + t_pt;
    Eigen::Vector3d Pr = q_ref.inverse() * (Pw - t_ref);
    pcl::PointXYZI p;
    p.x = Pr.x(); p.y = Pr.y(); p.z = Pr.z(); p.intensity = pt.intensity;
    out_cloud.push_back(p);
  }
  return out_cloud;
}

// 保存点云为PCD文件，文件名为时间戳，保存在pointclouds目录，支持去畸变
void save_pointcloud_to_pcd_with_undistort(const sensor_msgs::PointCloud2 &pc_msg, double timestamp, const std::vector<TumPose> &enu_poses)
{

  // 保存到 work_dir/pointclouds/
  std::string dir = g_work_dir + "/pointclouds/";
  struct stat st;
  if (stat(dir.c_str(), &st) != 0)
  {
    mkdir(dir.c_str(), 0777);
  }
  static int id = 0;
  std::ostringstream oss;
  oss << dir << id++ << "_" << std::fixed << std::setprecision(3) << timestamp << ".pcd";
  std::string filename = oss.str();

  // std::cerr << "to save [PCD] : " << std::endl;

  pcl::PointCloud<PandarPointXYZIRT> raw_cloud;
  pcl::fromROSMsg(pc_msg, raw_cloud);
  pcl::PointCloud<pcl::PointXYZI> undistorted = undistort_pointcloud(raw_cloud, enu_poses, timestamp);

  // 保存未去畸变点云，文件名加o
  // std::string filename_o = filename;
  // size_t dot_pos = filename_o.rfind('.');
  // if (dot_pos != std::string::npos) {
  //   filename_o.insert(dot_pos, "o");
  // } else {
  //   filename_o += "o";
  // }
  // pcl::PointCloud<pcl::PointXYZI> raw_cloud_xyzi;
  // raw_cloud_xyzi.reserve(raw_cloud.size());
  // for (const auto& pt : raw_cloud) {
  //   pcl::PointXYZI p;
  //   p.x = pt.x; p.y = pt.y; p.z = pt.z; p.intensity = pt.intensity;
  //   raw_cloud_xyzi.push_back(p);
  // }
  // if (pcl::io::savePCDFile(filename_o, raw_cloud_xyzi) == 0) {
  //   std::cout << "[PCD] Saved (original): " << filename_o << std::endl;
  // } else {
  //   std::cerr << "[PCD] Failed to save: " << filename_o << std::endl;
  // }

  // 保存去畸变点云
  if (pcl::io::savePCDFile(filename, undistorted) == 0)
  {
    std::cout << "[PCD] Saved (undistorted): " << filename << std::endl;
  }
  else
  {
    std::cerr << "[PCD] Failed to save: " << filename << std::endl;
  }
}


int main(int argc, char **argv)
{
  // 队列和同步
  std::deque<GnssOdomData> gnss_queue;
  std::deque<LidarData> lidar_queue;
  std::mutex mtx;
  std::condition_variable cv;
  bool finished = false;

  // 读取配置文件
  std::string config_file = "/home/xf/Desktop/catkin_ws/src/HBA/rviz_cfg/config.yaml";

  if (argc > 1) config_file = argv[1];
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const std::exception &e) {
    std::cerr << "Failed to load config file: " << config_file << ", error: " << e.what() << std::endl;
    return 1;
  }
  std::string work_dir = config["paths"]["work_dir"].as<std::string>("/mnt/nvme0n1p2/data/nongan_m2_1028/");
  std::string temp_file_dir = work_dir + "/debug_file/";
  std::string gnss_enu = temp_file_dir + "/lidar_at_enu.tum";
  std::string utm_offset = temp_file_dir + "/utm_offset.yaml";
 
  g_work_dir = work_dir;

  // Ensure work_dir exists
  struct stat st;
  if (stat(work_dir.c_str(), &st) != 0) {
    if (mkdir(work_dir.c_str(), 0777) != 0) {
        std::cerr << "Failed to create work_dir: " << work_dir << std::endl;
        return 1;
    }
  }

  // Ensure temp_file_dir exists
  if (stat(temp_file_dir.c_str(), &st) != 0) {
    if (mkdir(temp_file_dir.c_str(), 0777) != 0) {
        std::cerr << "Failed to create temp_file_dir: " << temp_file_dir << std::endl;
        return 1;
    }
  }

  // 启动YAML对齐导出线程
  std::thread worker = start_yaml_worker(gnss_queue, lidar_queue, mtx, cv, finished);

  ros::init(argc, argv, "bag_gnss_to_tum");

  std::string bag_path = config["bag_path"] ? config["bag_path"].as<std::string>() : "";
  std::string gnss_topic = config["gnss_topic"] ? config["gnss_topic"].as<std::string>() : "";
  std::string lidar_topic = config["lidar_topic"] ? config["lidar_topic"].as<std::string>() : "";

  // 读取lidar_gnss_extrinsic外参（四元数+平移）
  Eigen::Quaterniond q_extr(1, 0, 0, 0);
  Eigen::Vector3d t_extr(0, 0, 0);
  if (config["lidar_gnss_extrinsic"]) {
    const auto& extrinsic = config["lidar_gnss_extrinsic"];
    if (extrinsic["quaternion"]) {
      q_extr.w() = extrinsic["quaternion"]["w"].as<double>(1.0);
      q_extr.x() = extrinsic["quaternion"]["x"].as<double>(0.0);
      q_extr.y() = extrinsic["quaternion"]["y"].as<double>(0.0);
      q_extr.z() = extrinsic["quaternion"]["z"].as<double>(0.0);
    }
    if (extrinsic["translation"]) {
      t_extr.x() = extrinsic["translation"]["x"].as<double>(0.0);
      t_extr.y() = extrinsic["translation"]["y"].as<double>(0.0);
      t_extr.z() = extrinsic["translation"]["z"].as<double>(0.0);
    }
  }

  if (bag_path.empty()) {
    std::cerr << "bag_path is required in config.yaml" << std::endl;
    return 1;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception &e) {
    ROS_ERROR("Failed to open bag: %s", e.what());
    return 1;
  }

  std::vector<std::string> topics;
  topics.push_back(gnss_topic);
  topics.push_back(lidar_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ENUConverter enu;
  std::vector<TumPose> enu_poses;
  std::vector<TumPose> key_lidar_poses_at_enu;
  std::vector<double> lidar_times;
  std::vector<Eigen::Vector3d> gnss_utm_positions; // store raw UTM x,y,z per GNSS
  std::vector<double> odom_times; // 记录所有odom时间戳

  std::cout << "Extrinsic (GNSS->LiDAR):\n  R = \n" << q_extr.toRotationMatrix() << "\n  t = [" << t_extr.transpose() << "]\n";
  // T_lidar = T_gnss * X (if X is expressed in GNSS frame to LiDAR)
  // Here we use right-multiplication on homogeneous: P_lidar = P_gnss * X
  
  // create directory if not exists
  if (!temp_file_dir.empty()) {
    // trailing slash normalization
    if (temp_file_dir.back() != '/' && temp_file_dir.back() != '\\') temp_file_dir += '/';
    struct stat st;
    if (stat(temp_file_dir.c_str(), &st) != 0) {
      mkdir(temp_file_dir.c_str(), 0777);
    }
  }
  int count_gnss = 0;
  for (const rosbag::MessageInstance &m : view) {
    count_gnss++;
    // if (count_gnss > 3000)
    // {
    //   break;
    // }
    std::cout << "Processing message #" << count_gnss << "\r" << std::flush;
    
    if (m.getTopic() == gnss_topic) {
      auto msg = m.instantiate<chcnav::hcinspvatzcb>();
      if (!msg) continue;
      double enu_x, enu_y, enu_z;
      enu.convertToENU(msg->latitude, msg->longitude, msg->altitude, enu_x, enu_y, enu_z);
      Eigen::Quaterniond q_g = rpyDegToQuat( msg->roll, msg->pitch, msg->yaw );
      auto t_g = Eigen::Vector3d(enu_x,enu_y,enu_z);
      Eigen::Quaterniond q_l = q_g * q_extr;
      Eigen::Vector3d p_l = t_g + q_g * t_extr;
      TumPose tp(q_l, p_l, msg->header.stamp.toSec());
      enu_poses.emplace_back(tp);
      odom_times.emplace_back(msg->header.stamp.toSec());
      // ...existing code for GnssOdomData, utm_poses, etc...
      GnssOdomData g;
      g.timestamp = msg->header.stamp.toSec();
      g.position = t_g;
      g.position_lla = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
      g.pose = q_g.toRotationMatrix();
      g.enu_velocity = Eigen::Vector3d(msg->enu_velocity.x, msg->enu_velocity.y, msg->enu_velocity.z);
      g.heading = msg->heading;
      g.speed = msg->speed;
      {
        std::lock_guard<std::mutex> lock(mtx);
        gnss_queue.push_back(g);
        cv.notify_one();
      }

    } else if (m.getTopic() == lidar_topic) {
      sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
      if (!pc) continue;
      double ts = pc->header.stamp.toSec();
      lidar_times.emplace_back(ts);
      // LiDAR数据入队
      LidarData l; l.timestamp = ts;
      {
        std::lock_guard<std::mutex> lock(mtx);
        lidar_queue.push_back(l);
        cv.notify_one();
      }

    }
  }


  // 1. 配对每个雷达时间戳最近的odom时间戳，形成配对集合，并一一保存
  struct LidarOdomPair {
    double lidar_ts;
    double odom_ts;
  };
  std::vector<LidarOdomPair> lidar_odom_pairs;
  for (double ts_lidar : lidar_times) {
    double min_dt = std::numeric_limits<double>::max();
    double best_odom = -1;
    for (double ts_odom : odom_times) {
      double dt = std::abs(ts_lidar - ts_odom);
      if (dt < min_dt) {
        min_dt = dt;
        best_odom = ts_odom;
      }
    }
    if (best_odom >= 0) {
      lidar_odom_pairs.push_back({ts_lidar, best_odom});
    }
  }

  // 2. 保存与雷达帧一一对应的odom帧
  for (const auto& pair : lidar_odom_pairs) {
    double odom_ts = pair.odom_ts;
    auto it = std::find_if(enu_poses.begin(), enu_poses.end(), [&](const TumPose& tp)
    { return std::abs(tp.timestamp - odom_ts) < 1e-2; });

    if (it == enu_poses.end()) continue;

    Eigen::Vector3d pos = it->t;
    Eigen::Matrix3d R = it->q.toRotationMatrix();
    geometry_msgs::Vector3 dummy_vel; dummy_vel.x = 0; dummy_vel.y = 0; dummy_vel.z = 0;
    g_saved_odom_ts.insert(odom_ts);   
  }

  // 3. 保存所有雷达帧对应的点云
  bag.close();
  rosbag::Bag bag2;
  bag2.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view2(bag2);

  for (const rosbag::MessageInstance &m : view2) {
    if (m.getTopic() == lidar_topic) {
      sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
      if (!pc) continue;
      double ts = pc->header.stamp.toSec();
      
      // 判断内存集合中是否有对应odom
      if (g_saved_odom_ts.find(ts) == g_saved_odom_ts.end()) {
        // 没有对应odom，不保存点云
        continue;
      }

      // 查找enu_poses中与ts最接近的一个
      auto it_pose = std::min_element(enu_poses.begin(), enu_poses.end(), [ts](const TumPose& a, const TumPose& b) {
        return std::abs(a.timestamp - ts) < std::abs(b.timestamp - ts);
      });
      if (it_pose != enu_poses.end()) {
        it_pose->timestamp = ts; // 强制同步时间戳
        key_lidar_poses_at_enu.push_back(*it_pose);
      }

      // 保存所有雷达帧点云（与odom一一对应）
      save_pointcloud_to_pcd_with_undistort(*pc, ts, enu_poses);
    }
  }
  bag2.close();

  if (enu_poses.empty()) {
    ROS_ERROR("No GNSS messages found on topic %s", gnss_topic.c_str());
    return 1;
  }
  
  // Save GNSS enu poses to TUM
  std::ofstream f(gnss_enu);
  if (!f.is_open())
  {
    ROS_ERROR("Cannot open output file: %s", gnss_enu.c_str());
    return 1;
  }
  for (const auto &tp : key_lidar_poses_at_enu)
  {
    f << std::fixed << std::setprecision(3) << tp.timestamp << " "
      << std::setprecision(6) << tp.t.x() << " " << tp.t.y() << " " << tp.t.z() << " "
      << tp.q.x() << " " << tp.q.y() << " " << tp.q.z() << " " << tp.q.w() << "\n";
  }
  f.close();
  ROS_INFO("Wrote %zu GNSS poses to %s", key_lidar_poses_at_enu.size(), gnss_enu.c_str());

  // Write ENU origin (lat, lon, alt and ENU origin coordinates) in YAML format
  YAML::Node node;
  if (!utm_offset.empty() && enu.initialized)
  {
    node["lat"] = enu.lat0;
    node["lon"] = enu.lon0;
    node["alt"] = enu.h0;

    double utm_x, utm_y;
    int utm_zone = 0;
    bool utm_northp = true;
    GeographicLib::UTMUPS::Forward(enu.lat0, enu.lon0, utm_zone, utm_northp, utm_x, utm_y);

    node["utm_offset"]["x"] = utm_x;
    node["utm_offset"]["y"] = utm_y;
    node["utm_offset"]["alt"] = enu.h0;
    node["utm_offset"]["zone"] = utm_zone;
    node["utm_offset"]["northp"] = utm_northp;

    std::ofstream(utm_offset) << node;
    ROS_INFO("Wrote ENU origin to %s (with UTM)", utm_offset.c_str());
  }
  else
  {
    node["error"] = "ENU origin not initialized ";
    ROS_ERROR("Wrote ENU origin to %s (with UTM) Failed ...... ", utm_offset.c_str());
    std::ofstream(utm_offset) << node;
  }
  // 通知工作线程结束
  {
    std::lock_guard<std::mutex> lock(mtx);
    finished = true;
    cv.notify_all();
  }
  worker.join();
  return 0;

}

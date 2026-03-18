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

#include <chcnav/hcinspvatzcb.h>

// PCL for saving pointclouds
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "common_func.cpp"

// 用于记录已保存的odom时间戳
#include <unordered_set>
#include <map>
#include <limits>
static std::unordered_set<double> g_saved_odom_ts;
static std::string g_work_dir = ".";

// ! 关键帧参数
const float THRESH_DISTENT = 2.0;
const float THRESH_YAW_DEGREE = 10.0;


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




// 用于记录所有odom和点云的配对关系
static std::map<double, int> key_frame_map; // lidar_ts -> odom_ts

// 查找key_frame_map中与ts最接近且差值小于0.05的时间戳和id
std::pair<double, int> get_ts_and_id(double ts, const std::map<double, int>& key_frame_map, double threshold = 0.05) {
  auto it = key_frame_map.lower_bound(ts);
  double min_diff = std::numeric_limits<double>::max();
  int id = -1;
  double id_ts = -1;
  if (it != key_frame_map.end()) {
    min_diff = std::abs(it->first - ts);
    id = it->second;
    id_ts = it->first;
  }
  if (it != key_frame_map.begin()) {
    auto it_prev = std::prev(it);
    double diff = std::abs(it_prev->first - ts);
    if (diff < min_diff) {
      min_diff = diff;
      id = it_prev->second;
      id_ts = it_prev->first;
    }
  }
  if (min_diff >= threshold) {
    return std::make_pair(-1, -1);
  }
  return std::make_pair(id_ts, id);
}


// 提取关键帧时间戳及其id，返回map<timestamp, id>，并去除enu_poses中非关键帧
std::map<double, int> get_key_frames_timestamps(std::vector<TumPose>& enu_poses) {
  std::map<double, int> ts2id;
  if (enu_poses.empty()) return ts2id;
  Eigen::Vector3d last_position = enu_poses[0].t;
  double last_yaw = enu_poses[0].q.toRotationMatrix().eulerAngles(2,1,0)[0] * 180.0 / M_PI;
  int id = 0;
  std::vector<size_t> key_indices;
  key_indices.push_back(0);
  ts2id[enu_poses[0].timestamp] = id++;
  for (size_t i = 1; i < enu_poses.size(); ++i) {
    const auto& tp = enu_poses[i];
    Eigen::Vector3d position = tp.t;
    double curr_yaw = tp.q.toRotationMatrix().eulerAngles(2,1,0)[0] * 180.0 / M_PI;
    double dist = (position - last_position).norm();
    double dyaw = std::fabs(curr_yaw - last_yaw);
    if (dyaw > 180.0) dyaw = 360.0 - dyaw;
    if (dist > THRESH_DISTENT || dyaw > THRESH_YAW_DEGREE) {
      double ts = tp.timestamp;
      ts2id[ts] = id++;
      key_indices.push_back(i);
      last_position = position;
      last_yaw = curr_yaw;
    }
  }
  // 保留关键帧
  std::vector<TumPose> filtered;
  for (size_t idx : key_indices) {
    filtered.push_back(enu_poses[idx]);
  }
  enu_poses = std::move(filtered);
  return ts2id;
}

// 将lidar_times中每个时间戳与enu_poses最近的元素配对，结果存入key_lidar_poses_at_enu
// ! try 恢复之前 小于 0.05s 的时间差阈值，避免错误配对      

void get_lidar_time_pose_pairs(const std::vector<double>& lidar_times, const std::vector<TumPose>& enu_poses, std::vector<TumPose>& key_lidar_poses_at_enu) {
  key_lidar_poses_at_enu.clear();
  if (enu_poses.empty()) return;
  const double max_dt = 0.005;

  for (double ts : lidar_times) {
    auto it = std::lower_bound(
      enu_poses.begin(), enu_poses.end(), ts,
      [](const TumPose& p, double t) { return p.timestamp < t; }
    );

    const TumPose* best = nullptr;

    // 优先使用时间戳较小的一侧
    if (it != enu_poses.begin()) {
      auto it_prev = std::prev(it);
      if (std::abs(it_prev->timestamp - ts) < max_dt) {
        best = &(*it_prev);
      }
    }

    // 若较小一侧没有满足阈值，再尝试较大一侧
    if (!best && it != enu_poses.end()) {
      if (std::abs(it->timestamp - ts) < max_dt) {
        best = &(*it);
      }
    }

    if (best) {
      TumPose pose = *best;
      pose.timestamp = ts; // 强制同步为lidar时间戳
      key_lidar_poses_at_enu.push_back(pose);
    }
  }
}

// Write a single pose YAML file using yaml-cpp (只保留写文件逻辑)
void write_odom_yaml(double timestamp, const GnssOdomData& g, int id) {
  std::string odom_dir = g_work_dir + "/odoms/";
  std::ostringstream yaml_fn;
  yaml_fn << odom_dir << id << "_" << std::fixed << std::setprecision(3) << timestamp << ".yaml";
  std::cout << "[YAML] Writing: " << yaml_fn.str() << std::endl;

  YAML::Node node;
  node["position"].push_back(g.position.x());
  node["position"].push_back(g.position.y());
  node["position"].push_back(g.position.z());
  {
    std::ostringstream ts_ss;
    ts_ss << std::fixed << std::setprecision(3) << timestamp;
    node["timestamp"] = ts_ss.str();
  }
  node["position_lla"].push_back(g.position_lla.x());
  node["position_lla"].push_back(g.position_lla.y());
  node["position_lla"].push_back(g.position_lla.z());
  node["pose"].push_back(g.pose(0,0)); node["pose"].push_back(g.pose(0,1)); node["pose"].push_back(g.pose(0,2));
  node["pose"].push_back(g.pose(1,0)); node["pose"].push_back(g.pose(1,1)); node["pose"].push_back(g.pose(1,2));
  node["pose"].push_back(g.pose(2,0)); node["pose"].push_back(g.pose(2,1)); node["pose"].push_back(g.pose(2,2));
  node["enu_velocity"].push_back(g.enu_velocity.x());
  node["enu_velocity"].push_back(g.enu_velocity.y());
  node["enu_velocity"].push_back(g.enu_velocity.z());
  node["heading"] = g.heading;
  node["speed"] = g.speed;
  std::ofstream fout(yaml_fn.str());
  fout << node;
  fout.close();
}

// 实现write_odoms函数
void write_odoms(const std::map<double, int>& key_frame_map, const std::deque<GnssOdomData>& gnss_queue) {
  for (const auto& kv : key_frame_map) {
    double ts = kv.first;
    int id = kv.second;
    // 在gnss_queue中查找对应时间戳的数据 （允许一定的时间误差，比如0.05秒）
    auto it = std::find_if(gnss_queue.begin(), gnss_queue.end(), [ts](const GnssOdomData& g) {
      return std::abs(g.timestamp - ts) < 5e-3;
    });
    if (it != gnss_queue.end()) {
      write_odom_yaml(ts, *it, id);
      g_saved_odom_ts.insert(ts);
    } else {
      std::cerr << "[write_odoms] No GNSS data found for ts=" << std::fixed << std::setprecision(3) << ts << std::endl;
    }
  }
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
  
  timestamp = std::floor(timestamp * 1000.0 + 0.5) / 1000.0;
  auto ts_id_pair = get_ts_and_id(timestamp, key_frame_map);
  timestamp = ts_id_pair.first;
  int id = ts_id_pair.second;
  if (id == -1) {
    return;
  }

  std::ostringstream oss;
  oss << dir << id << "_" << std::fixed << std::setprecision(3) << timestamp << ".pcd";
  // oss << dir << std::fixed << std::setprecision(3) << timestamp << ".pcd";
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
  // if (pcl::io::savePCDFileASCII(filename, undistorted) == 0)
  if (pcl::io::savePCDFileBinary(filename, undistorted) == 0)
  {
    std::cout << " NOTE [PCD] Saved (undistorted): " << filename << std::endl;
  }
  else
  {
    std::cerr << "[PCD] Failed to save: " << filename << std::endl;
  }
}

// void create_dir_if_not_exists(const std::string& dir) 
// {
//   struct stat st;
//   if (stat(dir.c_str(), &st) != 0) {
//     mkdir(dir.c_str(), 0777);
//   }
// }

int main(int argc, char **argv)
{
  // 队列和同步
  std::deque<GnssOdomData> gnss_queue;

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
  create_dir_if_not_exists( g_work_dir );
  create_dir_if_not_exists( temp_file_dir );
  create_dir_if_not_exists( g_work_dir + "/images/" );
  create_dir_if_not_exists( g_work_dir + "/odoms/" );
  create_dir_if_not_exists( g_work_dir + "/pointclouds/" );

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


  int count_gnss = 0;
  for (const rosbag::MessageInstance &m : view) {
    count_gnss++;
    // if (count_gnss > 2500)
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

      // 计算LiDAR在ENU下的位姿
      Eigen::Quaterniond q_l = q_g * q_extr;
      Eigen::Vector3d p_l = t_g + q_g * t_extr;

      double timestamp = msg->header.stamp.toSec();
      timestamp = std::floor(timestamp * 1000.0 + 0.5) / 1000.0;

      TumPose tp(q_l, p_l, timestamp);
      enu_poses.emplace_back(tp);
      odom_times.emplace_back(timestamp);
      // ...existing code for GnssOdomData, utm_poses, etc...
      GnssOdomData g;
      g.timestamp = timestamp;
      g.position = t_g;
      g.position_lla = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
      g.pose = q_g.toRotationMatrix();
      g.enu_velocity = Eigen::Vector3d(msg->enu_velocity.x, msg->enu_velocity.y, msg->enu_velocity.z);
      g.heading = msg->heading;
      g.speed = msg->speed;
      gnss_queue.push_back(g);
    } else if (m.getTopic() == lidar_topic) {
      sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
      if (!pc) continue;
      double ts = pc->header.stamp.toSec();
      lidar_times.push_back(ts);
    }
  }

  get_lidar_time_pose_pairs(lidar_times, enu_poses, key_lidar_poses_at_enu);

  std::cout << "key_lidar_poses_at_enu: " << key_lidar_poses_at_enu.size() << std::endl;
  key_frame_map = get_key_frames_timestamps(key_lidar_poses_at_enu);
  write_odoms(key_frame_map, gnss_queue);
  std::cout << "Total GNSS messages: " << enu_poses.size() << ", total key frames: " << key_frame_map.size() << std::endl;

  // 3. 保存所有雷达帧对应的点云
  bag.close();
  rosbag::Bag bag2;
  bag2.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view2(bag2);

  // count_gnss = 0;
  for (const rosbag::MessageInstance &m : view2) {
    // count_gnss++;
    // if (count_gnss > 3000)
    // {
    //   break;
    // }
    if (m.getTopic() == gnss_topic) {
      // 已经在第一轮处理过了，这里跳过
      continue;
    } 

    if (m.getTopic().find("camera") != std::string::npos) 
    {
      auto topic_name = m.getTopic();
      // 极简容错：先找两个/的位置，再判断是否有效
      size_t pos1 = topic_name.find('/');
      size_t pos2 = topic_name.find('/', pos1+1);
      std::string cam_name = "unknown";
      if (pos1 != std::string::npos && pos2 != std::string::npos) {
        cam_name = topic_name.substr(pos1+1, pos2-pos1-1);
      }

      sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
      if (!img) continue;

      double ts = img->header.stamp.toSec();
      ts = std::floor(ts * 1000.0 + 0.5) / 1000.0;
      auto ts_id_pair = get_ts_and_id(ts, key_frame_map);
      double id_ts = ts_id_pair.first;
      int img_id = ts_id_pair.second;
      if (img_id == -1) {
        continue;
      }

      // 保存图像到 work_dir/images/
      std::string dir = g_work_dir + "/images/" + cam_name + "/";
      create_dir_if_not_exists(dir);

      std::ostringstream oss;
      // ! jpg 更小
      // oss << dir << img_id << "_" << std::fixed << std::setprecision(3) << id_ts << ".png";
      oss << dir << img_id << "_" << std::fixed << std::setprecision(3) << id_ts << ".jpg";
      std::string filename = oss.str();
      // 转换为cv::Mat并保存
      try {
        cv::Mat mat;
        // std::cout << "[Image518] img->encoding: " << img->encoding << std::endl;
        if (img->encoding == "rgb8" || img->encoding == "bgr8") {
          mat = cv::Mat(img->height, img->width, img->encoding == "rgb8" ? CV_8UC3 : CV_8UC3, const_cast<uchar*>(&img->data[0]), img->step);
          if (img->encoding == "rgb8") {
            cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
          }
        } else if (img->encoding == "mono8") {
          mat = cv::Mat(img->height, img->width, CV_8UC1, const_cast<uchar*>(&img->data[0]), img->step);
        } else if (img->encoding == "bayer_rggb8") {
          cv::Mat bayer(img->height, img->width, CV_8UC1, const_cast<uchar*>(&img->data[0]), img->step);
          // cv::cvtColor(bayer, mat, cv::COLOR_BayerRG2BGR);
          cv::cvtColor(bayer, mat, cv::COLOR_BayerRG2RGB); // id4
        } else {
          std::cerr << "[Image] Unsupported encoding: " << img->encoding << std::endl;
          continue;
        }
        if (!cv::imwrite(filename, mat)) {
          std::cerr << "[Image] Failed to save: " << filename << std::endl;
        } else {
          // std::cout << "[Image] Saved: " << filename << std::endl;
        }
      } catch (const std::exception& e) {
        std::cerr << "[Image] Exception: " << e.what() << std::endl;
      }
    } else if (m.getTopic() == lidar_topic) {
      sensor_msgs::PointCloud2::ConstPtr pc = m.instantiate<sensor_msgs::PointCloud2>();
      if (!pc) continue;
      double ts = pc->header.stamp.toSec();
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

  write_enu_origin_yaml(enu, utm_offset);
 
  return 0;
}

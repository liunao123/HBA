#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <csignal>
#include <signal.h>
#include <limits>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <math.h>

#include <thread>
#include <pcl/common/transforms.h> // 必须包含这个头文件

struct PandarPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT( PandarPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

// struct PointXYZIT {
//   float x;
//   float y;
//   float z;
//   unsigned char intensity;
//   double timestamp;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
// } EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     PointXYZIT,
//     (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity,
//                                             intensity)(double, timestamp,
//                                                        timestamp))


typedef pcl::PointXYZI pointtype;
// typedef PandarPointXYZIRT pointtype_pandar;

using namespace std;
using namespace Eigen;

bool stop = false;
bool exit_flag = false;
size_t i = 0;
std::vector<double> st_pose;  // 添加缺失的时间戳向量
std::vector<std::string> pcd_name;  // 添加缺失的时间戳向量

struct pose
{
  pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
       Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)):q(_q), t(_t){}
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
};


int threadFunction()
{
    while ( !exit_flag )
    {
        // std::cout << "按下 Enter 键暂停循环，再次按下 Enter 键继续，或输入 'q' 退出：" << std::endl;
        int c = getchar();
        if (c == '\n')
        {
          stop = !stop;
          std::cout << "stop is " << stop << " . at : " << i <<  std::endl;
        }
        if (c == 'q')
        {
          exit_flag = true;
          std::cout << "exit ... ... " << std::endl;
          exit(0);
        }
        // ros::Duration(0.5).sleep();
         usleep(500 * 1000);
    }
    std::cout << "threadFunction exit ." << std::endl;
    return 0;
}

void signal_callback_handler(int signum)
{
  if (signum == SIGINT)
  {
    exit_flag = true;
    std::cout << "Caught signal, EXIT " << signum << std::endl;
    // Terminate program
    exit(signum);
  }
}

std::vector<pose> read_pose(std::string filename,
                            Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
                            Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
{
  std::vector<pose> pose_vec;
  std::fstream file;
  file.open(filename);
  double tx, ty, tz, w, x, y, z;
  double num, st;
  std::string header;
  std::cout << "pose filename is " << filename << std::endl;
  std::string line;

  while (getline(file, line))
  {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    istringstream iss(line);
    long double timestamp;
    double tx, ty, tz, qx, qy, qz, qw;
    if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)
    {
      // file >> num >> st >> tx >> ty >> tz >> x >> y >> z >> w;
      // file >> st >> tx >> ty >> tz >> x >> y >> z >> w;
      Eigen::Quaterniond q(qw, qx, qy, qz);
      Eigen::Vector3d t(tx, ty, tz);
      // pose_vec.push_back(pose(qe * q, qe * t + te));
      pose_vec.push_back(pose(q, t));
      st_pose.push_back(timestamp);
      pcd_name.push_back( std::to_string( timestamp ) );
      std::cout << "   timestamp:   " << std::to_string(  timestamp ) << std::endl;
    }
  }
  file.close();
  pose_vec.pop_back();
  std::cout << "pose size is " << pose_vec.size() << std::endl;
  return pose_vec;
}

int main(int argc, char** argv)
{
  signal(SIGINT, signal_callback_handler);
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh("~");

  std::thread myThread(threadFunction);
  myThread.detach();

  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 100);
  ros::Publisher pub_debug = nh.advertise<sensor_msgs::PointCloud2>("/cloud_debug", 100);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseArray>("/poseArrayTopic", 10);
  ros::Publisher pub_trajectory = nh.advertise<visualization_msgs::Marker>("/trajectory_marker", 100);
  ros::Publisher pub_pose_number = nh.advertise<visualization_msgs::MarkerArray>("/pose_number", 100);
  
  ros::Publisher pubLidarPose = nh.advertise<geometry_msgs::PoseStamped>("/pose_offline", 10);
  ros::Publisher pubSurfPoint = nh.advertise<sensor_msgs::PointCloud2>("/points_offline", 10);

  string data_path, pose_file, pcd_path;
  double downsample_size, marker_size;
  int pcd_name_fill_num = 6;
  int pcd_start_index = 0;
  int pcd_end_index = 0;
  bool save_global_map = false;
  int pub_step = 2;

  nh.getParam("pose_file", pose_file);
  nh.getParam("pcd_path", pcd_path);
  nh.getParam("downsample_size", downsample_size);
  nh.getParam("pcd_start_index", pcd_start_index);
  
  nh.getParam("marker_size", marker_size);
  nh.getParam("save_global_map", save_global_map);
  nh.getParam("pub_step", pub_step);
  ROS_WARN("pub_step %d ", pub_step);

  sensor_msgs::PointCloud2 debugMsg, cloudMsg, outMsg;
  vector<pose> pose_vec;

  pose_vec =  read_pose(pose_file);

  size_t pose_size = pose_vec.size();
  cout<<"pose size "<<pose_size<<endl;
  nh.getParam("pcd_end_index", pcd_end_index);

  if(pcd_end_index > pose_size)
    pcd_end_index = pose_size;

  cout<<"pcd_end_index "<< pcd_end_index <<endl;

  pcl::PointCloud<pointtype>::Ptr pc_surf(new pcl::PointCloud<pointtype>);
  // pcl::PointCloud<pointtype_pandar>::Ptr pc_surf_pandar(new pcl::PointCloud<pointtype_pandar>);
    pcl::PointCloud<pointtype>::Ptr pc_filtered_1(new pcl::PointCloud<pointtype>);

  ros::Time cur_t;
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "odom";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;

  // cout<<"push enter to view"<<endl;
  // getchar();
  usleep(2000*1000);
  // ros::Duration(2).sleep();
  pcl::PointCloud<pointtype> global_map;

  double range = 50.0;

  i = pcd_start_index;

  ros::Rate  rate(10);

  // Eigen::Vector3d offset(275000.0223369 , 3479281.54229995 , 0.0);
  Eigen::Vector3d offset = pose_vec[0].t;
    std::cout  << "   offset:   " << offset << std::endl;

  for( ; i < pcd_end_index ; i++)
  {
    // if ( stop )
    // {
    //   i--;
    //   continue;
    // }

    if (exit_flag)
    {
      break;
    }

    // 只使用部分帧
    if ( i % pub_step )
    {
      continue;
    }

    if ( i % 500 == 0 )
    {
      ROS_INFO("read %0.1f%% , %zuth file , total %d  . ", float(100.0*i / pcd_end_index) , i , pcd_end_index );
    }
    // if( i > 250  && i < 1600 ) continue;
    // if( i > 2000  && i < 3000 ) continue;
    // if( i > 3700  && i < 4000 ) continue;

    pc_surf->points.clear();
    // pc_surf_pandar->points.clear();
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << st_pose[i];
    std::string pcd_st = pcd_path + oss.str() + "_surf.pcd";
    // std::string pcd_st = pcd_path + pcd_name[i] + ".pcd";
    // std::string pcd_st = pcd_path + std::to_string( int(st_pose[i])) + ".pcd";
    std::cout << i << "   pcd_st: " << pcd_st << std::endl;

    if (pcl::io::loadPCDFile(pcd_st, *pc_surf) == -1)
    {
      ROS_ERROR("Failed to load PCD file: %s", pcd_st.c_str());
      ROS_ERROR("Exiting due to PCD loading failure at frame %ld", i);
      continue;
    }
    // Check if point cloud is empty
    if (pc_surf->points.empty())
    {
      ROS_WARN("Point cloud is empty at frame %zu, skipping...", i);
      continue;
    }

    Eigen::Matrix4d key_pose = Eigen::Matrix4d::Identity();
    // 将四元数转换为旋转矩阵并填充变换矩阵
    pose_vec[i].q.normalize();
    key_pose.block<3, 3>(0, 0) = pose_vec[i].q.toRotationMatrix();
    // key_pose.block<3, 3>(0, 0) = qq.toRotationMatrix();

    // 设置平移部分
    if ( std::fabs(pose_vec[i].t.y()) > 10.0  || std::fabs(pose_vec[i].t.x()) > 10.0  )
    {
      pose_vec[i].t -= offset;
    }

    key_pose.block<3, 1>(0, 3) = pose_vec[i].t;
    std::cout << i << "  key_pose: " << key_pose << std::endl;

    pcl::PointCloud<pointtype>::Ptr global_pts(new pcl::PointCloud<pointtype>);
    
    // Transform point cloud with error handling
    try {
        pcl::transformPointCloud(*pc_surf, *global_pts, key_pose);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during point cloud transformation at frame %zu: %s", i, e.what());
        continue;
    }
    // std::cout << i << "  275 qt: " << key_pose << std::endl;
    // std::cout << "qt: " << pose_vec[i].t.transpose()  << std::endl;
 
    if(save_global_map)
    {
      global_map += *global_pts;
    }

    // 根据位置去网格化
    // 会丢失RGB信息
    // downsample_voxel(*pc_filtered_1, downsample_size);

    pcl::toROSMsg(*global_pts, cloudMsg);
    cloudMsg.header.frame_id = "odom";
    cloudMsg.header.stamp = ros::Time().fromSec(st_pose[i]);
    pub_map.publish(cloudMsg);

    geometry_msgs::Pose apose;
    apose.orientation.w = pose_vec[i].q.w();
    apose.orientation.x = pose_vec[i].q.x();
    apose.orientation.y = pose_vec[i].q.y();
    apose.orientation.z = pose_vec[i].q.z();
    apose.position.x = pose_vec[i].t(0);
    apose.position.y = pose_vec[i].t(1);
    apose.position.z = pose_vec[i].t(2);
    parray.poses.push_back(apose);
    pub_pose.publish(parray);

    geometry_msgs::PoseStamped pst;
    pst.header.stamp = ros::Time().fromSec(st_pose[i]);
    pst.header.frame_id = "odom";    
    pst.pose = apose;
    pubLidarPose.publish(pst);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_vec[i].t(0), pose_vec[i].t(1), pose_vec[i].t(2)));
    tf::Quaternion q(pose_vec[i].q.x(), pose_vec[i].q.y(), pose_vec[i].q.z(), pose_vec[i].q.w());
    transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(st_pose[i]) , "odom", "base_link"));

    // publish pose trajectory
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = cur_t;
    marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = (std::to_string(i) + "_" + std::to_string( st_pose[i] ) ).c_str();
    marker.pose.position.x = pose_vec[i].t(0);
    marker.pose.position.y = pose_vec[i].t(1);
    marker.pose.position.z = pose_vec[i].t(2);
    pose_vec[i].q.normalize();
    marker.pose.orientation.x = pose_vec[i].q.x();
    marker.pose.orientation.y = pose_vec[i].q.y();
    marker.pose.orientation.z = pose_vec[i].q.x();
    marker.pose.orientation.w = pose_vec[i].q.w();
    marker.scale.x = marker_size; // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.y = marker_size;
    marker.scale.z = marker_size;
    marker.color.r = float(1-float(i)/pose_size);
    marker.color.g = float(float(i)/pose_size);
    marker.color.b = float(float(i)/pose_size);
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    pub_trajectory.publish(marker);

    // publish pose number
    visualization_msgs::Marker marker_txt;
    marker_txt.header.frame_id = "odom";
    marker_txt.header.stamp = cur_t;
    marker_txt.ns = "marker_txt";
    marker_txt.id = i; // Any marker sent with the same namespace and id will overwrite the old one
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ostringstream str;
    str << i;
    marker_txt.text = str.str();
    marker.action = visualization_msgs::Marker::ADD;
    marker_txt.action = visualization_msgs::Marker::ADD;
    marker_txt.pose.position.x = pose_vec[i].t(0)+marker_size;
    marker_txt.pose.position.y = pose_vec[i].t(1)+marker_size;
    marker_txt.pose.position.z = pose_vec[i].t(2);
    marker_txt.pose.orientation.x = 0; pose_vec[i].q.x();
    marker_txt.pose.orientation.y = 0; pose_vec[i].q.y();
    marker_txt.pose.orientation.z = 0; pose_vec[i].q.x();
    marker_txt.pose.orientation.w = 1.0;
    marker_txt.scale.x = marker_size;
    marker_txt.scale.y = marker_size;
    marker_txt.scale.z = marker_size;
    marker_txt.color.r = 1.0f;
    marker_txt.color.g = 1.0f;
    marker_txt.color.b = 1.0f;
    marker_txt.color.a = 1.0;
    marker_txt.lifetime = ros::Duration();
    if(i%5 == 0) markerArray.markers.push_back(marker_txt);
    pub_pose_number.publish(markerArray);
    usleep(1*1000);
    // rate.sleep();
  }
  ROS_WARN("pub end:");

  if( save_global_map && global_map.size() )
  {
    ROS_WARN("save map: %ld ", global_map.size() );
    pcl::io::savePCDFile("/home/tyjt/Desktop/ros_ws/global_map_voxel.pcd", global_map);

    // pcl::io::savePCDFile(data_path + "global_map.pcd", global_map);
    ROS_WARN("save all points done . " );

    // static pcl::VoxelGrid<pointtype> dsrgb;
    // dsrgb.setLeafSize( downsample_size,  downsample_size,  downsample_size );
    // dsrgb.setInputCloud(global_map.makeShared());
    // dsrgb.filter(global_map);
    // ROS_INFO("world rgb pts size after <%lfm> VoxelGrid: %ld .", downsample_size , global_map.points.size());

    // ROS_WARN("save map end:");
    // downsample_voxel(global_map, 0.05);
    // ROS_WARN("downsample_voxel save map: %ld ", global_map.size() );
    // pcl::io::savePCDFile( data_path +"global_map_5cm.pcd", global_map);
    // downsample_voxel(global_map, 0.1);
    // ROS_WARN("downsample_voxel save map: %ld ", global_map.size() );
    // pcl::io::savePCDFile("/home/tyjt/Desktop/ros_ws/global_map_voxel.pcd", global_map);
    ROS_WARN("save map end:");
  }
  // ros::Rate loop_rate(1);
  // while(ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  ROS_WARN("exit......");
  exit_flag = true;

  return 0;
}
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <csignal>
#include <signal.h>

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
#include <rosbag/bag.h>

#include "mypcl.hpp"
#include <thread>

// 选择性编译
// #define USE_RGB

#ifdef USE_RGBA
typedef pcl::PointXYZRGBA PointTypeXYZRGBI;
#else
typedef pcl::PointXYZI PointTypeXYZRGBI;
#endif

using namespace std;
using namespace Eigen;

bool stop = false;
bool exit_flag = false;
size_t i = 0;

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
        }
        // ros::Duration(0.5).sleep();
         usleep(500 * 1000);
    }
    std::cout << "threadFunction exit ." << std::endl;
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

void filter_points_intensity_percent(pcl::PointCloud<PointTypeXYZRGBI>::Ptr & pts)
{
    std::cout << "Original points: " << pts->points.size() << std::endl;
    
    // Count intensity occurrences
    std::map<int, int> intensity_count;
    for (const auto &pt : pts->points)
    {
        intensity_count[pt.intensity]++;
    }
    std::cout << "Unique intensity values: " << intensity_count.size() << std::endl;

    // Convert map to vector for sorting
    std::vector<std::pair<int, int>> intensity_vec(intensity_count.begin(), intensity_count.end());
    std::sort(intensity_vec.begin(), intensity_vec.end(), 
              [](const auto &a, const auto &b) { return a.second < b.second; });

    // Calculate threshold for bottom 20%
    size_t threshold_index = static_cast<size_t>(intensity_vec.size() * 0.2);
    std::set<int> intensities_to_remove;
    int total_count = 0;
    size_t i = 0;
    while ( total_count < pts->points.size() * 0.2 )
    {
        intensities_to_remove.insert(intensity_vec[i].first);
        total_count += intensity_vec[i].second;
        // std::cout << "intensity_vec[i].second : " << intensity_vec[i].second << std::endl;
        // std::cout << "total_count : " << total_count << std::endl;
        i++;
    }
    std::cout << "intensities_to_remove : " << intensities_to_remove.size() << std::endl;
    
    // for (size_t i = 0; i < threshold_index; ++i)
    // {
    //     intensities_to_remove.insert(intensity_vec[i].first);
    // }

    // Filter points
    pcl::PointCloud<PointTypeXYZRGBI>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeXYZRGBI>);
    for (const auto &pt : pts->points)
    {
        if (intensities_to_remove.find(pt.intensity) == intensities_to_remove.end())
        {
            filtered_cloud->points.push_back(pt);
        }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;

    // *pts = *filtered_cloud;

    std::cout << "Filtered points: " << filtered_cloud->points.size() << std::endl;
    // // *pts = *filtered_cloud;
    pcl::PassThrough<PointType> pass_1;
    pass_1.setInputCloud( filtered_cloud );
    pass_1.setFilterFieldName("intensity");
    pass_1.setFilterLimits( 148 , 152 );
    pass_1.setNegative( true );
    pass_1.filter( *pts );

    pass_1.setInputCloud( pts );
    pass_1.setFilterLimits( -5 , 1 );
    pass_1.filter( *pts );

    std::cout << "filter points: " << pts->points.size() << std::endl;

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

  string data_path;
  double downsample_size, marker_size;
  int pcd_name_fill_num = 6;
  int pcd_start_index = 0;
  int pcd_end_index = 0;
  bool save_global_map = false;
  int pub_step = 2;

  nh.getParam("data_path", data_path);
  nh.getParam("downsample_size", downsample_size);
  nh.getParam("pcd_name_fill_num", pcd_name_fill_num);
  nh.getParam("pcd_start_index", pcd_start_index);
  
  nh.getParam("marker_size", marker_size);
  nh.getParam("save_global_map", save_global_map);
  nh.getParam("pub_step", pub_step);
  ROS_WARN("pub_step %d ", pub_step);
  // system( std::string("rosparam set /use_sim_time  false").c_str() );

  sensor_msgs::PointCloud2 debugMsg, cloudMsg, outMsg;
  vector<mypcl::pose> pose_vec;

  std::ifstream file_HBA( data_path + "HBA_pose.txt" );
  std::ifstream file_GTSAM( data_path + "GTSAM_pose.txt" );
  std::ifstream file_key_pose( data_path + "key_pose.txt" );
  if ( file_HBA.good() )
  {
    pose_vec = mypcl::read_pose(data_path + "HBA_pose.txt");
    ROS_WARN("read %sHBA_pose.txt", data_path.c_str());
  }
  else if(file_GTSAM.good())
  {
    pose_vec = mypcl::read_pose(data_path + "GTSAM_pose.txt");
    ROS_WARN("read %sGTSAM_pose.txt", data_path.c_str());
  }
  else if(file_key_pose.good())
  {
    pose_vec = mypcl::read_pose(data_path + "key_pose.txt");
    ROS_WARN("read %skey_pose.txt", data_path.c_str());
  }
  else
  {
    ROS_WARN(" can not read pose file . try to read %sposeGraph/graph.g2o " , data_path.c_str() );
    pose_vec = mypcl::readPosesFromG2O(data_path + "poseGraph/graph.g2o");
    // return -1;
  }
  
  std::vector<double> st_pose = mypcl::get_pose_stamp();

  size_t pose_size = pose_vec.size();
  cout<<"pose size "<<pose_size<<endl;
  nh.getParam("pcd_end_index", pcd_end_index);

  if(pcd_end_index > pose_size)
    pcd_end_index = pose_size;

  cout<<"pcd_end_index "<< pcd_end_index <<endl;

  pcl::PointCloud<PointTypeXYZRGBI>::Ptr pc_surf(new pcl::PointCloud<PointTypeXYZRGBI>);

  ros::Time cur_t;
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "odom";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;

  // cout<<"push enter to view"<<endl;
  // getchar();
  usleep(2000*1000);
  // ros::Duration(2).sleep();
  pcl::PointCloud<PointTypeXYZRGBI> global_map;

  float range = 3.0;
  pcl::CropBox<PointTypeXYZRGBI> cropBoxFilter_temp(false); //保留内部
  // pcl::RadiusOutlierRemoval<PointTypeXYZRGBI> outrem;
  // outrem.setRadiusSearch(0.2);
  // outrem.setMinNeighborsInRadius(1);

  range = 50.0;
  cropBoxFilter_temp.setMin(Eigen::Vector4f(-range, -range , -range, 1.0f));
  cropBoxFilter_temp.setMax(Eigen::Vector4f(range, range, range, 1.0f));
  i = pcd_start_index;

  ros::Rate  rate(10);

  for( ; i < pcd_end_index ; i++)
  {
    if ( stop )
    {
      i--;
      continue;
    }

    if (exit_flag)
    {
      break;
    }

    // 只使用部分帧
    if ( i % pub_step )
    {
      continue;
    }

    if ( i % 100 == 0 )
    {
      ROS_INFO("read %0.1f% , %ldth file , total %ld  . ", float(100.0*i / pcd_end_index) , i , pcd_end_index );
    }

    if( i > 1  && i < pcd_end_index ) 
    {
      if( std::fabs( pose_vec[i].t(2) - pose_vec[i-1].t(2) ) > 0.1 && std::fabs( pose_vec[i].t(2) - pose_vec[i+1].t(2) ) > 0.1)
      {
        continue;
      }
    }

    // if( i > 800  && i < 850 ) continue;

    if( i > 3310  && i < 3690 ) continue;


    // if( i > 3590  && i <  ) continue;
    // if( i > 5000  && i < 5130 ) continue;
    // if( i > 6000  && i < 6130 ) continue;
    // if( i > 6950  && i < 7000 ) continue;
    // if( i > 7800 ) continue;


    // if( i > 4500  && i < 4800 ) continue;
    // if( i > 5530  && i < 5600 ) continue;
    // if( i > 6500  && i < 6600 ) continue;
    // if( i > 7400  && i < 7500 ) continue;

    pc_surf->points.clear();
    
    // mypcl::loadPCD(data_path + "poseGraph/", pcd_name_fill_num, pc_surf, i );

    std::stringstream ss;
    if (pcd_name_fill_num > 0)
      ss << std::setw(pcd_name_fill_num) << std::setfill('0') << i;
    else
      ss << i;
    std::string pcd_st = data_path + "poseGraph/" + ss.str() + "/cloud.pcd";
    pcl::io::loadPCDFile(pcd_st, *pc_surf);

    // ROS_WARN("pc_surf : %d ", pc_surf->points.size() );
    // 第 0 个点云 应该没有 ，对应的位姿 是 0
    if( pc_surf->points.empty() )
    {
      continue;
    }

    pcl::PointCloud<PointTypeXYZRGBI>::Ptr pc_filtered(new pcl::PointCloud<PointTypeXYZRGBI>);
    pc_filtered->resize(pc_surf->points.size());

    // cropBoxFilter_temp.setNegative(true);  // 保留 range 之外的 点
    cropBoxFilter_temp.setNegative(false);  // 保留 range 之内的 点
    cropBoxFilter_temp.setInputCloud(pc_surf);
    cropBoxFilter_temp.filter(*pc_filtered);
    // *pc_filtered = *pc_surf;


    // filter_points_intensity_percent(pc_filtered);


    // pcl::io::savePCDFile("/opt/csg/slam/navs/test1.pcd", *pc_surf);
    //   continue;


    // outrem.setInputCloud(pc_filtered);
    // // apply filter
    // outrem.filter(*pc_filtered);
    // pose_vec[i].t(2) = 0;
    mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);

    // Eigen::Matrix4d key_pose = Eigen::Matrix4d::Identity();
    // // 将四元数转换为旋转矩阵并填充变换矩阵
    // //  pose_vec[i].q.normalize();
    // key_pose.block<3, 3>(0, 0) = pose_vec[i].q.toRotationMatrix();
    // // key_pose.block<3, 3>(0, 0) = qq.toRotationMatrix();
    // // 设置平移部分
    // key_pose.block<3, 1>(0, 3) = pose_vec[i].t;
    // pcl::transformPointCloud(*pc_filtered, *pc_filtered, key_pose);
    // std::cout << i << "qt: " << key_pose << std::endl;
    // std::cout << "qt: " << pose_vec[i].t.transpose()   << " " << qq.coeffs().transpose() << std::endl;


    if(save_global_map)
    {
      global_map += *pc_filtered;
    }

    // 根据位置去网格化
    // 会丢失RGB信息
    // downsample_voxel(*pc_filtered, downsample_size);

    pcl::toROSMsg(*pc_filtered, cloudMsg);
    cloudMsg.header.frame_id = "odom";
    cloudMsg.header.stamp = cur_t;
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

    pcl::toROSMsg(*pc_surf, cloudMsg);
    cloudMsg.header.stamp = ros::Time().fromSec(st_pose[i]);
    cloudMsg.header.frame_id = "base_link";    
    pubSurfPoint.publish(cloudMsg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_vec[i].t(0), pose_vec[i].t(1), pose_vec[i].t(2)));
    tf::Quaternion q(pose_vec[i].q.x(), pose_vec[i].q.y(), pose_vec[i].q.z(), pose_vec[i].q.w());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(st_pose[i]) , "odom", "base_link"));

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

    // 把强度信息恢复出来
    #ifdef USE_RGBA
    {
        pcl::PointCloud<pcl::PointXYZI> pc_pti;
        for (auto pt : global_map.points)
        {
            pcl::PointXYZI pti;
            pti.x = pt.x;
            pti.y = pt.y;
            pti.z = pt.z;
            pti.intensity = pt.a;
            pc_pti.points.push_back(pti);

            pt.a = 255; // 恢复成原样
        }
        pc_pti.width = 1;
        pc_pti.height = pc_pti.points.size();
        pcl::io::savePCDFile(data_path + "global_map_intensity.pcd", pc_pti);
    }
    #endif

    ROS_WARN("save map: %ld ", global_map.size() );
    pcl::io::savePCDFile(data_path + "global_map.pcd", global_map);
    ROS_WARN("save all points done . " );

    static pcl::VoxelGrid<PointTypeXYZRGBI> dsrgb;
    dsrgb.setLeafSize( downsample_size,  downsample_size,  downsample_size );
    dsrgb.setInputCloud(global_map.makeShared());
    dsrgb.filter(global_map);
    ROS_INFO("world rgb pts size after <%lfm> VoxelGrid: %ld .", downsample_size , global_map.points.size());

    // ROS_WARN("save map end:");
    // downsample_voxel(global_map, 0.05);
    // ROS_WARN("downsample_voxel save map: %ld ", global_map.size() );
    // pcl::io::savePCDFile( data_path +"global_map_5cm.pcd", global_map);
    // downsample_voxel(global_map, 0.1);
    // ROS_WARN("downsample_voxel save map: %ld ", global_map.size() );
    pcl::io::savePCDFile(data_path + "global_map_voxel.pcd", global_map);
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
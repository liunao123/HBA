#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>

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

#include "ba.hpp"
#include "tools.hpp"
#include "mypcl.hpp"
#include <thread>

using namespace std;
using namespace Eigen;

bool stop = false;
bool exit_flag = false;

int threadFunction()
{
    while (true)
    {
        if ( exit_flag )
        {
          break;
        }

        // std::cout << "按下 Enter 键暂停循环，再次按下 Enter 键继续，或输入 'q' 退出：" << std::endl;
        int c = getchar();
        if (c == '\n')
        {
          stop = !stop;
          std::cout << "stop is " << stop << std::endl;
        }
        if ( c == ' ')
        {
          std::cout << "break exit() " << std::endl;
          break;
        }
    }
}

int main(int argc, char** argv)
{
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
  system( std::string("rosparam set /use_sim_time  false").c_str() );

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
    ROS_WARN(" can not read pose file . try to read %spose_graph/graph.g2o " , data_path.c_str() );
    pose_vec = mypcl::readPosesFromG2O(data_path + "pose_graph/graph.g2o");
    // return -1;
  }
  
  std::vector<double> st_pose = mypcl::get_pose_stamp();

  size_t pose_size = pose_vec.size();
  cout<<"pose size "<<pose_size<<endl;
  nh.getParam("pcd_end_index", pcd_end_index);

  if(pcd_end_index > pose_size)
    pcd_end_index = pose_size;

  cout<<"pcd_end_index "<< pcd_end_index <<endl;

  pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_full(new pcl::PointCloud<pcl::PointXYZRGB>);

  ros::Time cur_t;
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "odom";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;

  // cout<<"push enter to view"<<endl;
  // getchar();
  usleep(5000*1000);
  pcl::PointCloud<PointType> global_map;

  float range = 2.0;
  pcl::CropBox<PointType> cropBoxFilter_temp(true);
  pcl::RadiusOutlierRemoval<PointType> outrem;
  outrem.setRadiusSearch(0.2);
  outrem.setMinNeighborsInRadius(1);

  range = 150.0;
  cropBoxFilter_temp.setMin(Eigen::Vector4f(-range, -range, -range, 1.0f));
  cropBoxFilter_temp.setMax(Eigen::Vector4f(range, range, 50, 1.0f));

  for(size_t i = pcd_start_index; i < pcd_end_index ; i++)
  {
    if ( stop )
    {
      i--;
      continue;
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

    // if( i > 1000  && i < 4000 ) continue;

    // if( i > 3020  && i < 3090 ) continue;
    // if( i > 3930  && i < 4140 ) continue;
    // if( i > 5000  && i < 5130 ) continue;
    // if( i > 6000  && i < 6130 ) continue;
    // if( i > 6950  && i < 7000 ) continue;
    // if( i > 7800 ) continue;


    // if( i > 4500  && i < 4800 ) continue;
    // if( i > 5530  && i < 5600 ) continue;
    // if( i > 6500  && i < 6600 ) continue;
    // if( i > 7400  && i < 7500 ) continue;

    mypcl::loadPCD(data_path + "pose_graph/", pcd_name_fill_num, pc_surf, i );

    pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>);
    pc_filtered->resize(pc_surf->points.size());

    // cropBoxFilter_temp.setNegative(true);  // 保留 range 之外的 点
    cropBoxFilter_temp.setNegative(false);  // 保留 range 之内的 点
    cropBoxFilter_temp.setInputCloud(pc_surf);
    cropBoxFilter_temp.filter(*pc_filtered);

    outrem.setInputCloud(pc_filtered);
    // apply filter
    outrem.filter(*pc_filtered);
    // pose_vec[i].t(2) = 0;
    mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t, pose_vec[i].q);

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
    pst.pose = apose;
    pubLidarPose.publish(pst);
    pcl::toROSMsg(*pc_surf, cloudMsg);
    cloudMsg.header.stamp = ros::Time().fromSec(st_pose[i]);
    pubSurfPoint.publish(cloudMsg);

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(pose_vec[i].t(0), pose_vec[i].t(1), pose_vec[i].t(2)));
    // tf::Quaternion q(pose_vec[i].q.x(), pose_vec[i].q.y(), pose_vec[i].q.z(), pose_vec[i].q.w());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "turtle_name"));

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
    if(i%GAP == 0) markerArray.markers.push_back(marker_txt);
    pub_pose_number.publish(markerArray);

    ros::Duration(0.001).sleep();
  }
  ROS_WARN("pub end:");

  if( save_global_map && global_map.size() )
  {
    ROS_WARN("save map: %ld ", global_map.size() );
    pcl::io::savePCDFile(data_path + "global_map.pcd", global_map);

    static pcl::VoxelGrid<PointType> dsrgb;
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
    pcl::io::savePCDFile( data_path + "global_map_voxel.pcd", global_map);
    ROS_WARN("save map end:");
  }

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (exit_flag)
    {
      break;
    }
  }
  
}
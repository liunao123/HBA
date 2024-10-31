#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <csignal>

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

// #include "ba.hpp"
// #include "tools.hpp"
#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh("~");

  ros::Publisher pubLidarPose = nh.advertise<geometry_msgs::PoseStamped>("/pose_offline", 10);

  // slam得到的雷达位姿 tum 格式
  vector<mypcl::pose> pose_vec;
  pose_vec = mypcl::read_pose("/home/map/test_office/lidar_pose_3dgs.txt");
  std::vector<double> st_pose = mypcl::get_pose_stamp();

  size_t pose_size = pose_vec.size();

  // 雷达与相机的外参 
  // Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
  // R_cl << 0.0161775, -0.999735, -0.0163792,
  //     -0.0438769, 0.0156558, -0.998914,
  //     0.998906, 0.0168786, -0.043612;
  // Eigen::Vector3d tt_cl;
  // tt_cl << -0.0242812, -0.0718288, -0.0921109;

  Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
  R_cl <<    0.0000000,  1.0000000,  0.0000000,
  -1.0000000,  0.0000000,  0.0000000,
   0.0000000,  0.0000000,  1.0000000;
  Eigen::Vector3d tt_cl(0,0,0);


  Eigen::Isometry3d T_cl = Eigen::Isometry3d::Identity();
  T_cl.rotate(R_cl);
  T_cl.pretranslate(tt_cl);
  std::cout << T_cl.matrix() << std::endl
            << std::endl;

  // 旋转部分转成四元数
  Eigen::AngleAxisd rotation_vector;
  rotation_vector.fromRotationMatrix(R_cl);
  Eigen::Quaterniond q_cl(rotation_vector);

  // 把点云 从雷达（第一帧）坐标系转到 相机坐标系（第一帧）
  pcl::PointCloud<PointType>::Ptr rgb_map(new pcl::PointCloud<PointType>);
  pcl::io::loadPCDFile( "/home/dlvc_data/vel2camera_vanjee_hk/xf_yl/33.pcd" , *rgb_map );
  mypcl::transform_pointcloud(*rgb_map, *rgb_map, tt_cl, q_cl );  //  
  pcl::io::savePCDFile( "/home/dlvc_data/vel2camera_vanjee_hk/xf_yl/3.pcd", *rgb_map);

  return 1;

  //! get inverse for colmap to do 3dgs
  // 第一帧 相机的位姿(雷达第一帧坐标系下)  其实就是 外参 T_lc
  Eigen::Quaterniond first_c_pose_q = q_cl.inverse();
  Eigen::Vector3d first_c_pose_t = -(q_cl.inverse() * tt_cl);

  int index = 1;
  for( const auto pose : pose_vec )
  {
    Eigen::Quaterniond q = pose.q;
    Eigen::Vector3d t = pose.t;

    // 由 雷达位姿获取相机位姿
    Eigen::Quaterniond q_c = q * q_cl.inverse();
    Eigen::Vector3d t_c = q * ( - (q_cl.inverse() * tt_cl) ) + t;
    // Eigen::Vector3d t_c = q * ( T_cl.inverse().translation() ) + t;

    //  相机的位姿全部统一到第一帧相机的位姿上
    // 复杂版本
    // q = first_c_pose_q.inverse() * q_c;
    // t = first_c_pose_q.inverse() * ( t_c - first_c_pose_t );

    // 简化版本
    q = q_cl * q_c;
    t = q_cl * t_c +  tt_cl ;

    // 可视化 
    // 可以看到传感器的运动过程
    geometry_msgs::Pose cam_pose;
    cam_pose.orientation.w = q.w();
    cam_pose.orientation.x = q.x();
    cam_pose.orientation.y = q.y();
    cam_pose.orientation.z = q.z();
    cam_pose.position.x = t.x();
    cam_pose.position.y = t.y();
    cam_pose.position.z = t.z();

    geometry_msgs::PoseStamped pst1;
    pst1.header.stamp = ros::Time().fromSec(st_pose[ index ]);
    pst1.header.frame_id = "odom";
    pst1.pose = cam_pose;

    pubLidarPose.publish(pst1);
    ros::Duration(0.03).sleep();

    // 计算 3dgs 需要的数据
    // 第一帧相机的位姿 相对后续每一帧位姿的坐标 <逆>
    auto q_inverse = q.inverse();
    // 不要用 auto 自动类型推导 第一种是对的，第二种自动推导的不对
    Eigen::Vector3d t_inverse = - ( q_inverse * t ) ;
    // Eigen::Vector3d t_inverse1 = - (q_inverse.matrix() * t);  ok
    // auto t_inverse1 = - (q_inverse.matrix() * t);   // ! NOT ok 

    std::cout << index << " " << q_inverse.w() << " " << q_inverse.x()  << " " << q_inverse.y() << " " << q_inverse.z()
              << " " << t_inverse.x()  << " " << t_inverse.y() << " " << t_inverse.z() << " 1 " << std::to_string(index) << ".jpg" 
              << std::endl << std::endl;

    index++;
  }

  cout<<"pose size "<<pose_size<<endl;
  
  return 1;

}
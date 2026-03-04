#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>
#include <string>
#include <iostream>
#include "common.hpp"
#include "file_utils.hpp"
#include <algorithm>


int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_pose_and_pts");
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/chcnav/odom", 10);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/hesai/pandar_points", 10);

    tf2_ros::TransformBroadcaster tf_broadcaster;
	// 指定要遍历的文件夹路径
	std::string folder = "/media/xf/Elements/id4_1202/raw_data_3/"; // 可根据需要修改
	// std::string points_folder = folder + "/pointclouds/"; 
	std::string points_folder = folder + "/pointclouds/"; 
	std::string odoms_folder = folder + "/odoms/"; 
	std::vector<std::string> pcd_files = getFilesWithExtension(points_folder, ".pcd");
	// std::vector<std::string> odoms_files = getFilesWithExtension(odoms_folder, ".yaml");
	ROS_INFO("Found %zu pcd files.", pcd_files.size());
    
    std::string tum_odom_file = folder + "/debug_file/lidar_at_enu.tum"; 
    std::vector<TumPose> tum_odoms = readTumPose( tum_odom_file );
	ROS_INFO("Found %zu tum_odoms files .", tum_odoms.size());

    ros::Rate rate(1); // 1Hz
    for (size_t i = 0; i < tum_odoms.size(); i++)
    {
        rate.sleep();
        ros::spinOnce();
        // 发布odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time().fromSec( tum_odoms[i].timestamp );
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "lidar";
        odom_msg.pose.pose.position.x = tum_odoms[i].t.x();
        odom_msg.pose.pose.position.y = tum_odoms[i].t.y();
        odom_msg.pose.pose.position.z = tum_odoms[i].t.z();
        odom_msg.pose.pose.orientation.x = tum_odoms[i].q.x();
        odom_msg.pose.pose.orientation.y = tum_odoms[i].q.y();
        odom_msg.pose.pose.orientation.z = tum_odoms[i].q.z();
        odom_msg.pose.pose.orientation.w = tum_odoms[i].q.w();
        odom_pub.publish(odom_msg);
        // 发布点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files[i], *cloud) == -1) //* 读取PCD            
        {
            PCL_ERROR("Couldn't read file %s \n", pcd_files[i].c_str());
            continue;
        }
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time().fromSec( tum_odoms[i].timestamp );
        cloud_msg.header.frame_id = "lidar";  
        cloud_pub.publish(cloud_msg);

        auto& tum_odom = tum_odoms[i];  
        std::cout << "odom: " << std::to_string(tum_odom.timestamp) << " " << tum_odom.t.transpose() << " " << tum_odom.q.coeffs().transpose() << std::endl;
        std::cout << "PCD: " << pcd_files[i] << " , size: " << cloud->size() << std::endl;

        // 发布tf
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time().fromSec(tum_odom.timestamp);
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "lidar";
        tf_msg.transform.translation.x = tum_odom.t.x();
        tf_msg.transform.translation.y = tum_odom.t.y();
        tf_msg.transform.translation.z = tum_odom.t.z();
        tf_msg.transform.rotation.x = tum_odom.q.x();
        tf_msg.transform.rotation.y = tum_odom.q.y();
        tf_msg.transform.rotation.z = tum_odom.q.z();
        tf_msg.transform.rotation.w = tum_odom.q.w();
        tf_broadcaster.sendTransform(tf_msg);
    }
    

    // for (const auto &pcd_path : pcd_files)
    // {
    //     // 这里后续将实现：构造yaml路径，加载pose，发布点云和pose
    //     std::cout << "PCD: " << pcd_path << std::endl;
    // }

    return 0;
}


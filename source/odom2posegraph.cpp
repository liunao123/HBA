#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <iostream>
#include <string>
#include <vector>
#include <csignal>

// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv/cv.hpp>

// #include <pcl/filters/voxel_grid.h>
// #include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

// typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointXYZI  PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

bool exit_flag = false;
int vertex_id = 0;
std::string  data_path = "./";

std::ofstream outfile ;
std::ofstream outfile_edge ;

void signal_callback_handler(int signum)
{
    exit_flag = true;
    std::cout << "Caught signal, EXIT " << signum << std::endl;
    // Terminate program

    exit(signum);
    ros::shutdown();
}


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // static std::ofstream outfile("./pose_graph_f.g2o", std::ios::app);
    // static std::ofstream outfile_edge("./pose_graph_edge_f.g2o", std::ios::app);
    static nav_msgs::Odometry::ConstPtr last_odom = msg;

    static const double period_time = 0.1;
    
    outfile.open("/home/pose_graph.g2o", std::ios::app);
    outfile_edge.open("/home/pose_graph_edge.g2o", std::ios::app);

    if (vertex_id == 0)
    {
        outfile << "VERTEX_SE3:QUAT " << vertex_id << " 0 0 0 0 0 0 1" << std::endl;
    }

    // Write vertex
    outfile << "VERTEX_SE3:QUAT " << vertex_id + 1 << " "
            << std::fixed << std::setprecision(6)
            << msg->pose.pose.position.x << " "
            << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " "
            << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " "
            << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << std::endl;

    // child_frame_id 坐标系下的线速度 得到对应的位移增量
    double delta_pt_x = msg->twist.twist.linear.x * period_time;
    double delta_pt_y = msg->twist.twist.linear.y * period_time;
    double delta_pt_z = msg->twist.twist.linear.z * period_time;
    Eigen::Vector3d delta_p(delta_pt_x, delta_pt_y, delta_pt_z);

    //
    // Eigen::Quaterniond q_w(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // 转到odom frame下
    // Eigen::Vector3d delta_p_l = q_w.inverse() * delta_p;
    // std::cout << delta_p_l.transpose() << std::endl;
    // std::cout << "---------------------------两种表达效果一样" << std::endl;
    // std::cout << (q_w.inverse().matrix() * delta_p).transpose() << std::endl << std::endl;
    //

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(msg->twist.twist.angular.x * period_time, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(msg->twist.twist.angular.y * period_time, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(msg->twist.twist.angular.z * period_time, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond delta_quaternion_ypr = rollAngle * pitchAngle * yawAngle;

    outfile_edge << "EDGE_SE3:QUAT " << vertex_id << " " << vertex_id + 1 << " "
                 << std::fixed << std::setprecision(6)
                 << delta_p.x() << " " << delta_p.y() << " " << delta_p.z() << " "
                 << delta_quaternion_ypr.x() << " " << delta_quaternion_ypr.y() << " " << delta_quaternion_ypr.z() << " " << delta_quaternion_ypr.w()
                 << " 1000 0 0 0 0 0 1000 0 0 0 0 1000 0 0 0 4000 0 0 4000 0 4000"
                 << std::endl;

    // 第 0 个点云 应该没有 ，对应的位姿 是 0
    vertex_id++;

    Eigen::Quaterniond quat_now(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix3d m3d_now = quat_now.matrix();

    Eigen::Quaterniond quat_last(last_odom->pose.pose.orientation.w, last_odom->pose.pose.orientation.x, last_odom->pose.pose.orientation.y, last_odom->pose.pose.orientation.z);
    Eigen::Matrix3d m3d_last = quat_last.matrix();
    
    Eigen::Vector3d t_last(last_odom->pose.pose.position.x, last_odom->pose.pose.position.y, last_odom->pose.pose.position.z);
    Eigen::Vector3d last_delta_p( last_odom->twist.twist.linear.x, last_odom->twist.twist.linear.y, last_odom->twist.twist.linear.z ) ;
    last_delta_p = last_delta_p * period_time;

    Eigen::Vector3d t_est =  quat_last * delta_p +  t_last ;
    
    outfile.close();
    outfile_edge.close();

    // save data file
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << vertex_id;
    std::string one_path = data_path + "pose_graph/" + ss.str();
    // ROS_WARN( "data_path is %s " , one_path.c_str() );
    system(("mkdir -p " + one_path).c_str());
    std::ofstream pose_data( one_path + "/data", std::ios::out);
    pose_data
        << "stamp " << int( msg->header.stamp.sec  ) << " " << int( msg->header.stamp.nsec ) << std::endl // 一定要是 int 的 time
        <<  "estimate" << std::endl << std::fixed << std::setprecision(10)
        // << m3d_now(0) << " " << m3d_now(1) << " " << m3d_now(2) << " " << msg->pose.pose.position.x << std::endl
        // << m3d_now(3) << " " << m3d_now(4) << " " << m3d_now(5) << " " << msg->pose.pose.position.y << std::endl
        // << m3d_now(6) << " " << m3d_now(7) << " " << m3d_now(8) << " " << msg->pose.pose.position.z << std::endl
        
        << m3d_last(0) << " " << m3d_last(1) << " " << m3d_last(2) << " " << t_est(0) << std::endl
        << m3d_last(3) << " " << m3d_last(4) << " " << m3d_last(5) << " " << t_est(1) << std::endl
        << m3d_last(6) << " " << m3d_last(7) << " " << m3d_last(8) << " " << t_est(2) << std::endl

        << "0 0 0 1" << std::endl
        << "odom" << std::endl
        // << m3d_last(0) << " " << m3d_last(1) << " " << m3d_last(2) << " " << t_est(0) << std::endl
        // << m3d_last(3) << " " << m3d_last(4) << " " << m3d_last(5) << " " << t_est(1) << std::endl
        // << m3d_last(6) << " " << m3d_last(7) << " " << m3d_last(8) << " " << t_est(2) << std::endl
        << m3d_now(0) << " " << m3d_now(1) << " " << m3d_now(2) << " " << msg->pose.pose.position.x << std::endl
        << m3d_now(3) << " " << m3d_now(4) << " " << m3d_now(5) << " " << msg->pose.pose.position.y << std::endl
        << m3d_now(6) << " " << m3d_now(7) << " " << m3d_now(8) << " " << msg->pose.pose.position.z << std::endl
        << "0 0 0 1" << std::endl
        // << "accum_distance -1" << std::endl
        << "id " << vertex_id << std::endl;
    pose_data.close();

    // Update vertex id and last pose
    last_odom = msg;
}

void ptsCallback(const sensor_msgs::PointCloud2::ConstPtr &pts)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pts, *cloud);

    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << vertex_id;
    std::string one_path = data_path + "pose_graph/" + ss.str();
    // system(("mkdir -p " + one_path).c_str());
    pcl::io::savePCDFile(one_path + "/cloud.pcd", *cloud);

}

void odom_pts_callback(const nav_msgs::Odometry::ConstPtr &odom, const sensor_msgs::PointCloud2::ConstPtr &pts)
{
    odomCallback(odom);
    ptsCallback(pts);
    ROS_WARN( "vertex_id is %d " , vertex_id );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2posegraph");

    signal(SIGINT, signal_callback_handler);

    std::string rm_cmd =  "rm /home/pose_graph.g2o  /home/pose_graph_edge.g2o ";
    system( rm_cmd.c_str());

    ros::NodeHandle nh;
    nh.getParam("data_path", data_path);
    sleep(3); // second s
    ROS_WARN(" remove %s .", data_path.c_str());
    system(("rm -r " + data_path).c_str());
    system(("mkdir -p " + data_path + "pose_graph/").c_str());

    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/undistort_laser", 10000);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/lidar_odom", 10000);

    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy_pts_img;
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy_pts_img;

    // 创建消息同步器，并将订阅器和回调函数绑定到同步器上
    message_filters::Synchronizer<MySyncPolicy_pts_img> sync_odom_pts(MySyncPolicy_pts_img(10000), odom_sub, pcl_sub);
    sync_odom_pts.registerCallback(boost::bind(&odom_pts_callback, _1, _2));

    while (ros::ok())
    {
      ros::spinOnce();
    }

    std::string g2o_path = data_path + "pose_graph/graph.g2o";
    std::string new_cmd =  "cat /home/pose_graph.g2o >>  " + g2o_path;
    // new_cmd = new_cmd +  " &&  echo \"\" >> " + g2o_path;
    new_cmd = new_cmd +  " &&  cat /home/pose_graph_edge.g2o >>  " + g2o_path;

    system( new_cmd.c_str());
    // ROS_WARN( "Cmd is %s" , new_cmd.c_str() );
    std::cout << "cmd is : " << new_cmd << std::endl;

    signal_callback_handler( 2 );

    return 0;
}
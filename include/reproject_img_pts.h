#ifndef PROJECT_H
#define PROJECT_H
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <std_srvs/Trigger.h>

#include <iostream>
#include <iomanip>
#include <numeric>
#include <vector>
#include <string>
#include <deque>
#include <mutex>
#include <algorithm>
#include <termios.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Dense>

typedef pcl::PointXYZRGB     PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

namespace DetectandTract{

    class projector{
        private:
        cv::Mat image_proj;
        image_transport::Publisher image_publisher;
        ros::Publisher pubLaserCloudFullRes;
        ros::Subscriber subPose;
        ros::ServiceServer server_save_rgbmap;

        PointCloudXYZRGB world_rgb_pts;
        PointCloudXYZRGB::Ptr rgb_pts_cloud;
        std::deque<geometry_msgs::PoseStamped::ConstPtr>  pose_buffer;
        std::mutex mtx_buffer;

        //crop lidar points
        float maxX = 150.0, maxY = 50.0, minZ = -20;
        float rgb_map_size = 0.05;
        int keyframe_cnts = 0;
        std::string data_path;
        bool dense_map = false;

        struct initial_parameters
        {
            /* data */
            std::string camera_topic;
            std::string pts_topic;
            std::string pose_topic;
            cv::Mat camtocam_mat;     
            cv::Mat cameraIn;
            cv::Mat RT;
            double cam_d0 , cam_d1, cam_d2, cam_d3;
            int cam_width, cam_height;
        }i_params;

        bool save_rgb_map_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
 

        void projection_img_pts_callback(const sensor_msgs::Image::ConstPtr &img, 
                                 const sensor_msgs::PointCloud2::ConstPtr &pc );

        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

        void initParams(ros::NodeHandle &nh);
        void matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform & trans);
        public:
        projector();


    };



}

#endif
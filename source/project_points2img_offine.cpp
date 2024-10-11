#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <csignal>
#include <thread>

#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv/cv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
bool exit_flag = false;

struct initial_parameters
{
    /* data */
    std::string camera_topic;
    std::string pts_topic;
    std::string pose_topic;
    cv::Mat camtocam_mat;
    cv::Mat cameraIn;
    cv::Mat RT;
    double cam_d0, cam_d1, cam_d2, cam_d3;
    int cam_width, cam_height;
} i_params;

 
void initParams(ros::NodeHandle &nh)
{
    double_t camtocam[12] = {0.0};
    double_t cameraIn[16] = {0.0};
    double_t RT[16] = {0.0};

    // parameter from ros
    nh.param<std::string>("camera_topic", i_params.camera_topic, "/image");
    nh.param<std::string>("pts_topic", i_params.pts_topic, "/undistort_laser");
    nh.param<std::string>("pose_topic", i_params.pose_topic, "/lidar_pose");

    // std::cout << "camera_topic: " << i_params.camera_topic << std::endl;
    // std::cout << "pts_topic: " << i_params.pts_topic << std::endl;
    // std::cout << "pose_topic: " << i_params.pose_topic << std::endl;

    double cam_fx = 0, cam_fy = 0, cam_cx = 0, cam_cy = 0;
    nh.param<double>("cam_fx", cam_fx, 453.483063);
    nh.param<double>("cam_fy", cam_fy, 453.254913);
    nh.param<double>("cam_cx", cam_cx, 318.908851);
    nh.param<double>("cam_cy", cam_cy, 234.238189);
    cameraIn[0] = cam_fx;
    cameraIn[5] = cam_fy;
    cameraIn[2] = cam_cx;
    cameraIn[6] = cam_cy;
    cameraIn[10] = 1.0;
    cameraIn[15] = 1.0;
    cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn); // cameratocamera params
    std::cout << __FILE__ << ":" << __LINE__ << std::endl << i_params.cameraIn << std::endl;

    // 定义相机的畸变参数
    nh.param<double>("cam_d0", i_params.cam_d0, -0.0971610);
    nh.param<double>("cam_d1", i_params.cam_d1, 0.1481190);
    nh.param<double>("cam_d2", i_params.cam_d2, -0.0017345);
    nh.param<double>("cam_d3", i_params.cam_d3, 0.0006040);
    nh.param<int>("cam_width", i_params.cam_width, 512);
    nh.param<int>("cam_height", i_params.cam_height, 612);
    std::cout << __FILE__ << ":" << __LINE__ << " distortionCoeffs: " << std::endl;
    std::cout << i_params.cam_d0 << std::endl;
    std::cout << i_params.cam_d1 << std::endl;
    std::cout << i_params.cam_d2 << std::endl;
    std::cout << i_params.cam_d3 << std::endl;
    std::cout << "cam_width : " << i_params.cam_width << std::endl;
    std::cout << "cam_height : " << i_params.cam_height << std::endl;

    std::vector<double> cameraextrinT(3, 0.0);
    std::vector<double> cameraextrinR(9, 0.0);
    std::cout << " try to get camera/Pcl and camera/Rcl param:" << std::endl;

    nh.param<std::vector<double>>("camera/Pcl", cameraextrinT, std::vector<double>());
    nh.param<std::vector<double>>("camera/Rcl", cameraextrinR, std::vector<double>());
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            RT[i * 4 + j] = cameraextrinR[i * 3 + j];
        }
    }
    RT[3] = cameraextrinT[0];
    RT[7] = cameraextrinT[1];
    RT[11] = cameraextrinT[2];
    RT[15] = 1;
    cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT); // lidar to camera params
    std::cout << __FILE__ << ":" << __LINE__ << std::endl
              << i_params.RT << std::endl;
}


const std::pair< PointCloudXYZRGB::Ptr , cv::Mat >  projection_img_pts_callback( const std::string pcd_filename, const std::string img_filename )
{
    cv::Mat raw_img = cv::imread(img_filename, cv::IMREAD_COLOR);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
    }
    // 对图像去畸变
    // 载入图像
    cv::Mat distortedImage = raw_img.clone();

    // 相机内参
    cv::Mat cameraMatrix = i_params.cameraIn(cv::Rect(0, 0, 3, 3));
    // std::cout << cameraMatrix << std::endl;
    cv::Mat distortionCoeffs = (cv::Mat_<double>(1, 4) << i_params.cam_d0, i_params.cam_d1, i_params.cam_d2, i_params.cam_d3);
    // std::cout << distortionCoeffs << std::endl;

    // 进行畸变矫正
    cv::Mat undistortedImage;
    cv::undistort(distortedImage, undistortedImage, cameraMatrix, distortionCoeffs);

    // 显示矫正后的图像
    // cv::namedWindow("Undistorted Image", cv::WINDOW_NORMAL);
    // cv::imshow("Undistorted Image", undistortedImage);
    // cv::waitKey(10);
    // cv::destroyAllWindows();
    
    // cv::Mat overlay = raw_img.clone();
    cv::Mat overlay = undistortedImage.clone();

    PointCloudXYZRGB::Ptr rgb_pts_cloud(new PointCloudXYZRGB(cloud->points.size(), 1));  // 预留的内存
    // rgb_pts_cloud->clear(); //会清空之前预留的内存

    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(4, 1, cv::DataType<double>::type);

    float maxX = 50;
    float maxY = 30;
    float minZ = -20;

    std::cout << __FILE__ <<":" << __LINE__ << " points size:  " << rgb_pts_cloud->points.size() << std::endl;

    int rgb_cnts = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++)
    {
        // if (it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ)
        if (it->x > maxX || it->x < 0.0 || it->z < minZ)
        {
            continue;
        }

        if ( std::fabs( std::atan2(it->y, it->x) ) > M_PI_4  )  // -45 to 45 degree
        {
            continue;
        }

        // if(it->intensity > 0.98 )
        // {
        //     continue;
        // }

        X.at<double>(0, 0) = it->x;
        X.at<double>(1, 0) = it->y;
        X.at<double>(2, 0) = it->z;
        X.at<double>(3, 0) = 1;

        Y = i_params.cameraIn * i_params.RT * X; // tranform the point to the camera coordinate

        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);
        // std::cout << "pixel: " << pt.y << " " << pt.x << std::endl;

        // 移除边缘的点
        const int remove_pixel_thres = 2;
        if (pt.y < remove_pixel_thres || pt.y > (i_params.cam_height - remove_pixel_thres) ||
            pt.x < remove_pixel_thres || pt.x > (i_params.cam_width - remove_pixel_thres))
        {
            continue;
        }

        PointTypeRGB pointRGB;
        // 点的位置不变
        pointRGB.x = it->x;
        pointRGB.y = it->y;
        pointRGB.z = it->z;

        // 获取该点对应的RGB值
        // cv::Vec3b pixel_rgb =  raw_img.at<cv::Vec3b>(pt.y, pt.x);
        cv::Vec3b pixel_rgb = undistortedImage.at<cv::Vec3b>(pt.y, pt.x);
        pointRGB.r = pixel_rgb[2];
        pointRGB.g = pixel_rgb[1];
        pointRGB.b = pixel_rgb[0];

        // float Gray = 0.2989 * pointRGB.r + 0.5870 * pointRGB.g + 0.1140 * pointRGB.b;
        // if ( Gray > 240.0  ) // 过白的点，就不要了
        // {
        // //   continue;
        //   pointRGB.r = 240;
        //   pointRGB.g = 240;
        //   pointRGB.b = 240;
        // }

        // rgb_pts_cloud->push_back(pointRGB);
        rgb_pts_cloud->points[rgb_cnts++]  = pointRGB ;

        if (rgb_cnts % ( cloud->points.size() / 100000 ) )
        // if ( rgb_cnts % 10000 )
        {
            continue;
        }

        float val = it->x;
        int red = std::min(255, (int)(255 * abs((val - maxX) / maxX)));
        int green = std::min(255, (int)(255 * (1 - abs((val - maxX) / maxX))));
        cv::circle(overlay, pt, 3, cv::Scalar(0, green, red), -1);
    }
    std::cout << __FILE__ <<":" << __LINE__ << " rgb_cnts:  " << rgb_cnts << std::endl;
    rgb_pts_cloud->points.resize(rgb_cnts);
    rgb_pts_cloud->width = rgb_cnts;
    rgb_pts_cloud->height = 1;

    // return rgb_pts_cloud;
    return std::make_pair( rgb_pts_cloud, overlay );
}

void signal_callback_handler(int signum)
{
  exit_flag = true;
  std::cout << "Caught signal, EXIT " << signum << std::endl;
  exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_points_to_image_offine");
    
    signal(SIGINT, signal_callback_handler);

    std::string data_path = "/home/direct_l_v_calibrate_data_avia/cab_hall/pcd_png";
    int PCD_NUM = 1 ;

    ros::NodeHandle nh;
    initParams(nh);
    
    nh.param<std::string>("data_path",data_path, "/home/map/rgb_test" );
    ROS_WARN("data_path is %s . ", data_path.c_str() );

    nh.param<int>("PCD_NUM", PCD_NUM, 2);
    ROS_WARN("img and pcd num is %d  . ", PCD_NUM );

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/rgb_pts", 10000);

    image_transport::ImageTransport imageTransport(nh);
    image_transport::Publisher image_publisher = imageTransport.advertise("/project_pc_image", 10000);

    for (size_t i = 0; i < PCD_NUM; i++ )
    {
        std::string image_file = data_path + "/" + std::to_string(i)  + ".png" ;
        std::string pcd_file = data_path + "/" + std::to_string(i)  + ".pcd" ;
        std::string rgb_pcd_file = data_path + "/" + std::to_string(i)  + "_rgb.pcd" ;
        ROS_WARN("loading %s . ", pcd_file.c_str());

        try
        {
            auto rgb_ponts_and_img = projection_img_pts_callback(pcd_file, image_file);
            pcl::io::savePCDFile(rgb_pcd_file, *( rgb_ponts_and_img.first ) );
            ROS_WARN("saving %s . then publish rgb pts. ", rgb_pcd_file.c_str());
            sensor_msgs::PointCloud2 wpts;
            // 上色后的点云 坐标系没变 lidar
            pcl::toROSMsg(*( rgb_ponts_and_img.first ), wpts);
            wpts.header.frame_id = "livox";
            wpts.header.stamp = ros::Time::now();
            pubLaserCloudFullRes.publish(wpts);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",  rgb_ponts_and_img.second ).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = "camera";
            ROS_INFO("time %lf", msg->header.stamp.toSec());  
            image_publisher.publish( *msg );

        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        sleep(5); // second s

        if (exit_flag)
        {
            break;
        }
    }

    return 0;
}
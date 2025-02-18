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
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

using namespace cv;   
using namespace std;   

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

const std::pair< PointCloudXYZI::Ptr , cv::Mat > generate_lidar_edge_points( const PointCloudXYZI::Ptr cloud, const cv::Mat cameraIn_4x4 , const cv::Mat T_cl_4x4 )
{
    cv::Mat intensity_image(i_params.cam_width, i_params.cam_height, CV_64FC1, cv::Scalar::all(0));
    cv::Mat index_image(i_params.cam_width, i_params.cam_height, CV_32SC1, cv::Scalar::all(-1));
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(4, 1, cv::DataType<double>::type);

    for (int it = 0; it < cloud->points.size(); it++)
    {
        X.at<double>(0, 0) = cloud->points[it].x;
        X.at<double>(1, 0) = cloud->points[it].y;
        X.at<double>(2, 0) = cloud->points[it].z;
        X.at<double>(3, 0) = 1;

        // 内参格式要统一 cv format
        Y = cameraIn_4x4 * T_cl_4x4 * X; // tranform the point to the camera coordinate

        cv::Point pt;
        pt.x = std::round (Y.at<double>(0, 0) / Y.at<double>(0, 2) );
        pt.y = std::round (Y.at<double>(0, 1) / Y.at<double>(0, 2) );

        // 移除边缘的点
        const int remove_pixel_thres = 2;
        if (pt.y < remove_pixel_thres || pt.y > (i_params.cam_height - remove_pixel_thres) ||
            pt.x < remove_pixel_thres || pt.x > (i_params.cam_width - remove_pixel_thres))
        {
            continue;
        }

        // 生成对应像素的强度
        // 记录对应这个像素点的雷达点索引
        intensity_image.at< double >(pt.y, pt.x) =  cloud->points[it].intensity;
        index_image.at< int >( pt.y, pt.x ) = it ;
    }

    cv::imwrite("/home/1.png", intensity_image);
    cv::Mat intensity_image_equa;

    cv::normalize(intensity_image, intensity_image_equa, 0, 255, cv::NORM_MINMAX);
    // cv::imwrite("/home/1_equa.png", intensity_image_equa);

    intensity_image_equa.clone().convertTo(intensity_image_equa, CV_8UC1, 1.0 );
    // cv::imwrite("/home/1_CV_8UC1.png", intensity_image_equa);

    cv::equalizeHist(intensity_image_equa, intensity_image_equa);
    cv::imwrite("/home/1_equa.png", intensity_image_equa);

    // cv::dilate(img, kernel, iteration);

    // 定义核（结构元素）用于膨胀和腐蚀
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // 膨胀处理
    cv::Mat dilated;
    cv::dilate(intensity_image_equa, dilated, element);

    // 提取强度图像的边缘
    cv::Mat edge;
    cv::Canny(dilated, edge, 150, 200);
    cv::imwrite("/home/intensity_image_equa_edge.png", edge);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( edge ,contours, hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_NONE  ,cv::Point());  
    std::cout << "159 contours.size: " << contours.size() << std::endl;

    cv::Mat imageContours=Mat::zeros(edge.size(),CV_8UC1);  
    cv::Mat S_Contours=Mat::zeros(edge.size(),CV_8UC1);  //绘制  
    for(int i=0;i<contours.size();i++)  
    {  
        if ( contours[i].size() < 100 )
        {
            continue;
        }
        
        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
        for(int j=0;j<contours[i].size();j++)   
        {  
            //绘制出contours向量内所有的像素点  
            Point P=Point(contours[i][j].x,contours[i][j].y);  
            S_Contours.at<uchar>(P)=255;  
        }  
        //输出hierarchy向量内容  
        // char ch[256];  
        // sprintf(ch,"%d",i);  
        // string str=ch;  
        // cout<<"向量hierarchy的第" << i <<" 个元素内容为：" << hierarchy[i] <<endl<<endl;  
        //绘制轮廓  
        drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);  
    } 
    // imshow("Contours Image",imageContours); //轮廓  
    // imshow("Point of Contours",S_Contours);   //向量contours内保存的所有轮廓点集  
    waitKey(0);

    // 根据图像的边缘来提取对应的雷达点云
    PointCloudXYZI::Ptr pts_edge(new PointCloudXYZI());
    for (int row = 0; row < S_Contours.rows; ++row) {
      for (int col = 0; col < S_Contours.cols; ++col) {
        // 访问每个元素
        uchar pixelValue = S_Contours.at<uchar>(row, col);
        // 处理 pixelValue
        if (pixelValue)
        {
          auto ind = index_image.at<int>(row, col);
          if (ind >= 0 && ind < cloud->points.size())
          {
            pts_edge->points.push_back(cloud->points[ind]);
          }
        }
      }
    }

    std::cout << "save lidar_edge done. pts: " << pts_edge->points.size() << std::endl;
    std::cout << "start RadiusOutlierRemoval: "  << std::endl;
    pcl::RadiusOutlierRemoval< PointType > outrem;
	outrem.setRadiusSearch(0.1);
	outrem.setMinNeighborsInRadius(5);
	// apply filter
	outrem.setInputCloud(pts_edge);
	outrem.filter(*pts_edge);
    std::cout << "save lidar_edge done. pts: " << pts_edge->points.size() << std::endl;
    
    pts_edge->width = pts_edge->points.size();
    pts_edge->height = 1;
    // pcl::io::savePCDFile("/home/lidar_edge.pcd", *pts_edge);
    // std::cout << "save lidar_edge done. pts: " << pts_edge->width << std::endl;
    // return pts_edge;
    return std::make_pair( pts_edge, edge );
}

void signal_callback_handler(int signum)
{
  exit_flag = true;
  std::cout << "Caught signal, EXIT " << signum << std::endl;
  exit(signum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_lidar_intensity_image");
    
    signal(SIGINT, signal_callback_handler);

    std::string data_path = "/home/direct_l_v_calibrate_data_avia/cab_hall/pcd_png";
    int PCD_NUM = 1 ;

    ros::NodeHandle nh;
    initParams(nh);
    
    nh.param<std::string>("data_path",data_path, "/home/map/rgb_test" );
    ROS_WARN("data_path is %s . ", data_path.c_str() );

    nh.param<int>("PCD_NUM", PCD_NUM, 2);
    ROS_WARN("img and pcd num is %d  . ", PCD_NUM );

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/edge_pts", 10000);

    image_transport::ImageTransport imageTransport(nh);
    image_transport::Publisher image_publisher = imageTransport.advertise("/lidar_intensity", 10000);

    for (size_t i = 7 ; i < PCD_NUM; i++ )
    {
        std::string pcd_file = data_path + "/" + std::to_string(i)  + ".pcd" ;
        std::string rgb_pcd_file = data_path + "/" + std::to_string(i)  + "_edge.pcd" ;
        std::string pts_img_file = data_path + "/" + std::to_string(i)  + "_edge_intensity.png" ;
        ROS_WARN("loading %s . ", pcd_file.c_str());

        try
        {
            PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
            if (pcl::io::loadPCDFile<PointType>(pcd_file, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file\n");
            }

            auto edge_pts_and_img = generate_lidar_edge_points(cloud, i_params.cameraIn, i_params.RT );

            pcl::io::savePCDFile(rgb_pcd_file, *( edge_pts_and_img.first ) );
            ROS_WARN("saving %s . then publish rgb pts. \n\n\n", rgb_pcd_file.c_str());
            sensor_msgs::PointCloud2 wpts;
            // 上色后的点云 坐标系没变 lidar
            pcl::toROSMsg(*( edge_pts_and_img.first ), wpts);
            wpts.header.frame_id = "livox";
            wpts.header.stamp = ros::Time::now();
            pubLaserCloudFullRes.publish(wpts);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",  edge_pts_and_img.second ).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = "camera";
            // ROS_INFO("time %lf", msg->header.stamp.toSec());
            image_publisher.publish( *msg );
            cv::imwrite(pts_img_file, edge_pts_and_img.second );
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
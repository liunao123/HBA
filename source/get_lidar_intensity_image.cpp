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

std::vector< Eigen::Vector2d > direct_vector;
std::vector< Eigen::Vector2d > centor_vector;

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

// Function to get pixel positions of a line segment using Bresenham's algorithm
std::vector<cv::Point> getLinePixels(const cv::Vec4i& line) {
    std::vector<cv::Point> pixels;
    cv::Point pt1(line[0], line[1]);
    cv::Point pt2(line[2], line[3]);

    // Bresenham's line algorithm
    int dx = std::abs(pt2.x - pt1.x);
    int dy = std::abs(pt2.y - pt1.y);
    int sx = (pt1.x < pt2.x) ? 1 : -1;
    int sy = (pt1.y < pt2.y) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        pixels.push_back(pt1);
        if (pt1.x == pt2.x && pt1.y == pt2.y) break;
        int err2 = err * 2;
        if (err2 > -dy) {
            err -= dy;
            pt1.x += sx;
        }
        if (err2 < dx) {
            err += dx;
            pt1.y += sy;
        }
    }
    return pixels;
}

void calcDirection(const std::vector<Eigen::Vector2d> &points, Eigen::Vector2d &direction)
{
    Eigen::Vector2d mean_point(0, 0);
    for (size_t i = 0; i < points.size(); i++)
    {
        mean_point(0) += points[i](0);
        mean_point(1) += points[i](1);
    }
    mean_point(0) = mean_point(0) / points.size();
    mean_point(1) = mean_point(1) / points.size();
    Eigen::Matrix2d S;
    S << 0, 0, 0, 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Matrix2d s = (points[i] - mean_point) * (points[i] - mean_point).transpose();
        S += s;
    }
    Eigen::EigenSolver<Eigen::Matrix<double, 2, 2>> es(S);
    Eigen::MatrixXcd evecs = es.eigenvectors();
    Eigen::MatrixXcd evals = es.eigenvalues();
    Eigen::MatrixXd evalsReal;
    evalsReal = evals.real();
    Eigen::MatrixXf::Index evalsMax;
    evalsReal.rowwise().sum().maxCoeff(&evalsMax); // 得到最大特征值的位置
    direction << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax);
    // std::cout << "evalsReal: " << evalsReal << std::endl;
    // std::cout << "direction: " << direction << std::endl;
    // if ( direction(0) < direction(1) )
    //   std::cout << "direction: " << direction(1) / direction(0) << std::endl;
    // else
    //   std::cout << "direction: " << direction(0) / direction(1) << std::endl;
}
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

bool isLine(const std::vector<Eigen::Vector2d>& points, double threshold = 0.95 ) {
    if (points.size() < 2) return false;

    // Calculate the centroid
    Eigen::Vector2d centroid(0, 0);
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= points.size();

    // Calculate the covariance matrix
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto& point : points) {
        Eigen::Vector2d centered = point - centroid;
        covariance += centered * centered.transpose();
    }

    // Perform eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    Eigen::Vector2d eigenvalues = solver.eigenvalues();

    Eigen::Matrix2d eigenvectors = solver.eigenvectors();
    // 最大特征值对应的特征向量
    Eigen::Vector2d lineDirection = eigenvectors.col(1); // 最大特征值对应的特征向量
    Eigen::Vector2d linePoint = centroid; // 线上的一个点是质心
    // 根据点到线的距离判断这些点是不是近似分布在一条线上

    double max_distance = 0;
    for (const auto &point : points)
    {
        // 计算从线上的点到给定点的向量
        Eigen::Vector2d v = point - linePoint;

        // 计算线方向的单位向量
        Eigen::Vector2d d = lineDirection.normalized();

        // 计算投影长度
        double projectionLength = v.dot(d);

        // 计算投影点
        Eigen::Vector2d projectionPoint = linePoint + projectionLength * d;

        // 计算距离
        double  distance =  (point - projectionPoint).norm();
        if ( distance > max_distance )
        {
            max_distance = distance;
        }
    }

    std::cout << "max_distance : " << max_distance << std::endl;
    std::cout << "direct_vector.size() : " << direct_vector.size() << std::endl;

    if ( max_distance > 2 )
    {
        return false;
    }

    // Check the ratio of the largest eigenvalue to the sum of eigenvalues
    double ratio = eigenvalues(1) / eigenvalues.sum();
    std::cout << "192 ratio : " << ratio << std::endl;

    if (ratio > threshold)
    {
        direct_vector.push_back(lineDirection);
        centor_vector.push_back(linePoint);
        return true;
    }
    else
        return false;
}

double pointToLineDistance(const Eigen::Vector3d& point, const Eigen::Vector3d& linePoint, const Eigen::Vector3d& lineDirection) {
    // 计算从线上的点到给定点的向量
    Eigen::Vector3d v = point - linePoint;

    // 计算线方向的单位向量
    Eigen::Vector3d d = lineDirection.normalized();

    // 计算投影长度
    double projectionLength = v.dot(d);

    // 计算投影点
    Eigen::Vector3d projectionPoint = linePoint + projectionLength * d;

    // 计算距离
    return (point - projectionPoint).norm();
}

void fit_3D_Line(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& linePoint, Eigen::Vector3d& lineDirection, bool& vail_line) {
    if ( points.size() < 5 )
    {
        vail_line = false;
        return ;
    }
    // 计算质心
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= points.size();

    // 计算协方差矩阵
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& point : points) {
        Eigen::Vector3d centered = point - centroid;
        covariance += centered * centered.transpose();
    }
    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // 最大特征值对应的特征向量
    lineDirection = eigenvectors.col(2); // 最大特征值对应的特征向量
    linePoint = centroid; // 线上的一个点是质心

    double max_distance = 0;
    for (const auto &point : points)
    {
        // 计算从线上的点到给定点的向量
        Eigen::Vector3d v = point - linePoint;

        // 计算线方向的单位向量
        Eigen::Vector3d d = lineDirection.normalized();

        // 计算投影长度
        double projectionLength = v.dot(d);

        // 计算投影点
        Eigen::Vector3d projectionPoint = linePoint + projectionLength * d;

        // 计算距离
        double  distance =  (point - projectionPoint).norm();
        if ( distance > max_distance )
        {
            max_distance = distance;
        }
    }

    std::cout << "307 max_distance : " << max_distance << std::endl;
    // std::cout << "308 direct_vector.size() : " << direct_vector.size() << std::endl;

    if ( max_distance > 0.15 )
        vail_line = false;
    else
        vail_line = true;

}


const std::pair< PointCloudXYZI::Ptr , cv::Mat > generate_lidar_edge_points( const PointCloudXYZI::Ptr cloud, const cv::Mat cameraIn_4x4 , const cv::Mat T_cl_4x4 )
{
    cv::Mat intensity_image(i_params.cam_width, i_params.cam_height, CV_64FC1, cv::Scalar::all(0));
    cv::Mat intensity_cnts(i_params.cam_width, i_params.cam_height, CV_64FC1, cv::Scalar::all(0));
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
        intensity_image.at< double >(pt.y, pt.x) +=  cloud->points[it].intensity;
        intensity_cnts.at< double >(pt.y, pt.x) ++ ;
        index_image.at< int >( pt.y, pt.x ) = it ;
    }

    // 强度取平均值
    for (int row = 0; row < intensity_image.rows; ++row)
    {
        for (int col = 0; col < intensity_image.cols; ++col)
        {
            if ( ! intensity_cnts.at< double >(row, col) )
            {
              intensity_image.at< double >(row, col) /= intensity_cnts.at< double >(row, col);
            }
        }
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
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // // 膨胀处理
    // cv::Mat dilated;
    // cv::dilate(intensity_image_equa, dilated, element);

    // // 提取强度图像的边缘
    cv::Mat edge;
    // cv::Canny(dilated, edge, 150, 200);
    // cv::imwrite("/home/intensity_image_equa_edge.png", edge);

    int gaussian_size = 5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_clouds(new pcl::PointCloud<pcl::PointXYZ>);
    // cv::GaussianBlur(intensity_image_equa, intensity_image_equa, cv::Size(gaussian_size, gaussian_size), 0, 0);
    cv::imwrite("/home/intensity_image_equa_edgeBlur.png", intensity_image_equa);
    cv::Canny(intensity_image_equa, edge, 150, 200, 3, true);
    cv::imwrite("/home/intensity_image_equa_edge.png", edge);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( edge ,contours, hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_NONE  ,cv::Point());  
    std::cout << "159 contours.size: " << contours.size() << std::endl;

    // Draw detected line segments
    cv::imshow("Line Segments", intensity_image_equa);
    cv::imwrite("/home/line_color_image.png", intensity_image_equa);
    // cv::waitKey(0);

    cv::Mat imageContours=Mat::zeros(edge.size(),CV_8UC1);  
    cv::Mat S_Contours=Mat::zeros(edge.size(),CV_8UC1);  //绘制  

    PointCloudXYZI::Ptr pts_edge(new PointCloudXYZI());

    for(int i=0;i<contours.size();i++)  
    {
        if ( contours[i].size() < 50 )
        {
            continue;
        }

        Eigen::Vector2d direction;
        std::vector<Eigen::Vector2d> points;

        // 同属一条线上的 3d 点
        // PointCloudXYZI::Ptr line_3d(new PointCloudXYZI());
        std::vector<Eigen::Vector3d> line_3d;

        // 计算这个轮廓是不是一条线
        for (int j = 0; j < contours[i].size(); j++)
        {
            Eigen::Vector2d pt(contours[i][j].x, contours[i][j].y);
            points.push_back(pt);

            auto ind = index_image.at<int>(contours[i][j].x, contours[i][j].y);
            if (ind >= 0 && ind < cloud->points.size())
            {
                // line_3d->points.push_back(cloud->points[ind]);
                Eigen::Vector3d pt( cloud->points[ind].x, cloud->points[ind].y, cloud->points[ind].z ) ;
                line_3d.push_back( pt );
            }
        }

        Eigen::Vector3d linePoint;
        Eigen::Vector3d lineDirection;
        bool vail_line = false;
        fit_3D_Line(line_3d, linePoint, lineDirection, vail_line);
        if ( !vail_line )
        {
            continue;
        }
        
        std::cout << "start add points to edge : "  << std::endl;
        for (const auto &pt : cloud->points)
        {
            Eigen::Vector3d pt_one(pt.x, pt.y, pt.z);
            auto dis = pointToLineDistance(pt_one, linePoint, lineDirection);
            if (dis < 0.05)
            {
              pts_edge->points.push_back( pt );
            }
        }
    }

    // imshow("Contours Image",imageContours); //轮廓  
    // imshow("Point of Contours",S_Contours);   //向量contours内保存的所有轮廓点集  
    // waitKey(0);
    // cv::imwrite("/home/S_Contours.png", S_Contours);
    // cv::imwrite("/home/imageContours.png", imageContours);

    // cv::Mat save_pixel(i_params.cam_width, i_params.cam_height, CV_64FC1, cv::Scalar::all(0));
    // cv::Mat save_pixel=Mat::zeros(edge.size(),CV_8UC1);  //绘制  
    // if (!direct_vector.empty())
    // {
    //         // centor_vector
    //     for (int row = 0; row < intensity_image_equa.rows; ++row)
    //     {
    //         for (int col = 0; col < intensity_image_equa.cols; ++col)
    //         {
    //             // 访问每个元素
    //             uchar pixelValue = intensity_image_equa.at<uchar>(row, col);
    //             // 处理 pixelValue
    //             if (pixelValue)
    //             {
    //                 for (size_t i = 0; i < direct_vector.size(); i++)
    //                 {
    //                     Eigen::Vector3d pt(row, col, 0);
    //                     auto dis = pointToLineDistance( pt, centor_vector[i], direct_vector[i] );
    //                     if ( dis < 2 )
    //                     {
    //                         // save_pixel.at<int>(row, col) = 200;
    //                         save_pixel.at<uchar>( col , row ) = 255;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
    // cv::imwrite("/home/save_pixel.png", save_pixel);

    // 根据图像的边缘来提取对应的雷达点云
    // PointCloudXYZI::Ptr pts_edge(new PointCloudXYZI());



    // for (int row = 0; row < edge.rows; ++row) {
    //   for (int col = 0; col < edge.cols; ++col) {
    //     // 访问每个元素
    //     uchar pixelValue = edge.at<uchar>(row, col);
    //     // 处理 pixelValue
    //     if (pixelValue)
    //     {
    //       auto ind = index_image.at<int>(row, col);
    //       if (ind >= 0 && ind < cloud->points.size())
    //       {
    //         pts_edge->points.push_back(cloud->points[ind]);
    //       }
    //     }
    //   }
    // }
    
    pts_edge->width = pts_edge->points.size();
    pts_edge->height = 1;
    pcl::io::savePCDFile("/home/lidar_edge_o.pcd", *pts_edge);

    // std::cout << "save lidar_edge done. pts: " << pts_edge->points.size() << std::endl;
    // std::cout << "start RadiusOutlierRemoval: "  << std::endl;
    // pcl::RadiusOutlierRemoval< PointType > outrem;
    // outrem.setRadiusSearch(0.2);
    // outrem.setMinNeighborsInRadius(10);
    // // apply filter
    // outrem.setInputCloud(pts_edge);
    // outrem.filter(*pts_edge);
    // std::cout << "save lidar_edge done. pts: " << pts_edge->points.size() << std::endl;
    
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

    for (size_t i = 2 ; i < PCD_NUM; i++ )
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
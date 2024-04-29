#include "reproject_img_pts.h"


namespace DetectandTract{
    void projector::initParams(ros::NodeHandle &nh)
    {
        double_t camtocam[12] = {0.0};
        double_t cameraIn[16] = {0.0};
        double_t RT[16] = {0.0} ;
             
        // parameter from ros
        nh.param<std::string>("camera_topic",i_params.camera_topic, "/image" );
        nh.param<std::string>("pts_topic",i_params.pts_topic, "/undistort_laser" );
        nh.param<std::string>("pose_topic",i_params.pose_topic, "/lidar_pose" );
        nh.param< float >("rgb_map_size",rgb_map_size, 0.05 );
        nh.param<std::string>("data_path",data_path, "/home/map/rgb_test" );
        nh.param< bool >("dense_map",dense_map, false );

        rgb_pts_cloud.reset(new PointCloudXYZRGB( ));

        ROS_ERROR("WARNING : after 10s will remove %s .", data_path.c_str());
        ros::Duration(5).sleep();
        ROS_ERROR(" remove %s .", data_path.c_str());
        
        system(("rm -r " + data_path + "./*").c_str());
        system(("mkdir -p " + data_path + "pose_graph/").c_str());

        // nh.getParam("camera_topic", i_params.camera_topic);

        std::cout << "camera_topic: " << i_params.camera_topic <<  std::endl;
        std::cout << "pts_topic: " << i_params.pts_topic <<  std::endl;
        std::cout << "pose_topic: " << i_params.pose_topic <<  std::endl;
        std::cout << "rgb_map_size: " << rgb_map_size <<  std::endl;

        double cam_fx = 0,  cam_fy = 0, cam_cx = 0, cam_cy = 0;
        nh.param<double>("cam_fx",cam_fx,453.483063);
        nh.param<double>("cam_fy",cam_fy,453.254913);
        nh.param<double>("cam_cx",cam_cx,318.908851);
        nh.param<double>("cam_cy",cam_cy,234.238189);
        cameraIn[0] = cam_fx;
        cameraIn[5] = cam_fy;
        cameraIn[2] = cam_cx;
        cameraIn[6] = cam_cy;
        cameraIn[10] = 1.0;
        cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);     //cameratocamera params
        std::cout << __FILE__ <<":" << __LINE__ <<  std::endl << i_params.cameraIn << std::endl;

        // 定义相机的畸变参数
        nh.param<double>("cam_d0",i_params.cam_d0, -0.0971610 );
        nh.param<double>("cam_d1",i_params.cam_d1,  0.1481190 );
        nh.param<double>("cam_d2",i_params.cam_d2, -0.0017345 );
        nh.param<double>("cam_d3",i_params.cam_d3,  0.0006040 );
        nh.param< int >("cam_width",i_params.cam_width, 512 );
        nh.param< int >("cam_height",i_params.cam_height, 612 );
        std::cout << __FILE__ <<":" << __LINE__ <<  " distortionCoeffs: " << std::endl;
        std::cout << i_params.cam_d0 <<  std::endl;
        std::cout << i_params.cam_d1 <<  std::endl;
        std::cout << i_params.cam_d2 <<  std::endl;
        std::cout << i_params.cam_d3 <<  std::endl;
        std::cout << "cam_width : " << i_params.cam_width <<  std::endl;
        std::cout << "cam_height : " << i_params.cam_height << std::endl;

        std::vector<double> cameraextrinT(3, 0.0);
        std::vector<double> cameraextrinR(9, 0.0);
        std::cout << " try to get camera/Pcl and camera/Rcl param:"   << std::endl;

        nh.param<std::vector<double>>("camera/Pcl", cameraextrinT, std::vector<double>());
        nh.param<std::vector<double>>("camera/Rcl", cameraextrinR, std::vector<double>());
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                RT[i * 4 + j] = cameraextrinR[i * 3 + j];
                // std::cout << cameraextrinR[i * 3 + j] << " , ";
            }
            // std::cout << std::endl;
        }
        RT[3] = cameraextrinT[0];
        RT[7] = cameraextrinT[1];
        RT[11] = cameraextrinT[2];
        cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT); // lidar to camera params
        std::cout << __FILE__ << ":" << __LINE__ << std::endl << i_params.RT << std::endl;
    }

    bool projector::save_rgb_map_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::cout << "save rgb map start ......" << std::endl;
        if (world_rgb_pts.points.empty())
        {
            res.success = false;
            res.message = "rgb pcd is empty -_- ";
            return true;
        }

        ROS_WARN("rgb points size %ld .", world_rgb_pts.points.size());

        pcl::io::savePCDFile(data_path + "livo_rgbmap_original.pcd",  world_rgb_pts);
        pcl::VoxelGrid<PointTypeRGB> dsrgb1;
        dsrgb1.setLeafSize(rgb_map_size, rgb_map_size, rgb_map_size);
        dsrgb1.setInputCloud(world_rgb_pts.makeShared());
        dsrgb1.filter(world_rgb_pts);
        ROS_WARN("rgb points size %ld .",  world_rgb_pts.points.size());

        pcl::io::savePCDFile(data_path + " livo_rgbmap.pcd",  world_rgb_pts);
        std::cout << "save rgb map done ......" << std::endl;
        res.success = true;
        res.message = "rgb pcd file: " + data_path + " livo_rgbmap.pcd" ;
        return true;
    }


    void projector::projection_img_pts_callback(const sensor_msgs::Image::ConstPtr &img,
                                        const sensor_msgs::PointCloud2::ConstPtr &pc)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat raw_img = cv_ptr->image;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*pc, *cloud);
        // ROS_WARN(" xyz pts size: %ld .", cloud->points.size());

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

        cv::Mat overlay = raw_img.clone();
        // std::cout<<"get img  data at time : " << std::to_string ( img->header.stamp.toSec()  ) <<std::endl;
        // std::cout<<"get pts  data at time : " << std::to_string ( pc->header.stamp.toSec()  ) <<std::endl;
        // std::cout<<"get pose data at time : " << std::to_string ( pose_msg->header.stamp.toSec()  ) <<std::endl;

        // PointCloudXYZRGB::Ptr rgb_pts_cloud(new PointCloudXYZRGB(cloud->points.size(), 1));
        rgb_pts_cloud->clear();

        cv::Mat X(4, 1, cv::DataType<double>::type);
        cv::Mat Y(3, 1, cv::DataType<double>::type);

        for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++)
        {
            if (it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ)
            {
                continue;
            }

            X.at<double>(0, 0) = it->x;
            X.at<double>(1, 0) = it->y;
            X.at<double>(2, 0) = it->z;
            X.at<double>(3, 0) = 1;

            Y = i_params.cameraIn * i_params.RT * X; // tranform the point to the camera coordinate

            // std::cout << __FILE__ <<":" << __LINE__ <<  std::endl << i_params.cameraIn << std::endl;
            // std::cout << __FILE__ <<":" << __LINE__ <<  std::endl << i_params.RT << std::endl;

            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);
            // std::cout << "pixel: " << pt.y << " " << pt.x << std::endl;

            // 移除边缘的点
            const int remove_pixel_thres = 10;
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

            float Gray = 0.2989 * pointRGB.r + 0.5870 * pointRGB.g + 0.1140 * pointRGB.b;
            if ( Gray > 245.0  ) // 过白的点，就不要了
            {
              pointRGB.r = 240;
              pointRGB.g = 240;
              pointRGB.b = 240;
            }

            rgb_pts_cloud->push_back(pointRGB);

            float val = it->x;
            float maxVal = 15.0;
            int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
            cv::circle(overlay, pt, 1, cv::Scalar(0, green, red), -1);
        }

        if ( rgb_pts_cloud->empty() )
        {
            ROS_ERROR("empty ");
            return;
        }
        // Publish the image projection
        // ros::Time time = ros::Time::now();
        cv_ptr->encoding = "bgr8";
        cv_ptr->header = img->header;
        cv_ptr->image = overlay;
        // cv_ptr->image =   undistortedImage  -  distortedImage; // undistortedImage; // debug for undistort img distortedImage - undistortedImage
        image_publisher.publish(cv_ptr->toImageMsg());
        // std::cout << "project picture is published!" << std::endl;

        // 判断是否有 pose 有的话就将rgb点云转到world坐标系
        if ( pose_buffer.empty() )
        {
            return;
        }
        else
        {
            // 找到与点云时间戳一样的pose
            if (pose_buffer.front()->header.stamp.toSec() > img->header.stamp.toSec())
            {
                return;
            }

            while (  !pose_buffer.empty()  )
            {
                if (pose_buffer.front()->header.stamp.toSec() == img->header.stamp.toSec())
                {
                    break;
                }
                else
                {
                    mtx_buffer.lock();
                    pose_buffer.pop_front();
                    mtx_buffer.unlock();
                }
            }
        }
        
        if ( pose_buffer.empty() )
        {
            return;
        }
        auto pose_msg = pose_buffer.front();

        ROS_WARN_ONCE("project pts with pose :");

        static Eigen::Matrix4d last_pose = Eigen::Matrix4d::Identity();

        // 按距离或者角度变化去决定是否保存该帧
        Eigen::Matrix4d transformMatrix_b2w = Eigen::Matrix4d::Identity();
        // 提取位姿的位置
        const auto &position = pose_msg->pose.position;
        Eigen::Vector3d translation(position.x, position.y, position.z);
        transformMatrix_b2w.block<3, 1>(0, 3) = translation;

        // 提取位姿的姿态
        const auto &orientation = pose_msg->pose.orientation;
        Eigen::Quaterniond quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
        quaternion.normalize(); // 归一化
        Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
        transformMatrix_b2w.block<3, 3>(0, 0) = rotationMatrix;

        auto delta_pose =  last_pose.inverse() * transformMatrix_b2w;    
        Eigen::Vector3d delta_translation = delta_pose.block<3, 1>(0, 3);
        float delta_yaw = std::atan2(delta_pose(1, 0), delta_pose(0, 0));
        // std::cout << "d_trans  : " << delta_translation.norm() << std::endl;
        // std::cout << "d_yaw : " << delta_yaw << std::endl;

        if (!dense_map)
        {
            if (delta_translation.norm() < 0.1 && std::fabs(delta_yaw) < 0.1 && keyframe_cnts != 0) // 0.1m or 0.1rad
            {
                return;
            }
        }

        // 更新试上一次的位姿
        last_pose = transformMatrix_b2w;

        std::string pcd_name = data_path + std::to_string(keyframe_cnts);
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << keyframe_cnts;
        std::string one_path = data_path + "pose_graph/" + ss.str();
        system(("mkdir -p " + one_path).c_str());

        pcl::io::savePCDFile(one_path + "/cloud.pcd", *rgb_pts_cloud);
        // pcl::io::savePCDFile(one_path + "/cloud_dis.pcd", *rgb_pts_cloud_dis);

        std::ofstream file((data_path + "pose_graph/graph.g2o"), std::ios_base::app); // 使用追加模式打开文件
        if (!file)
            std::cerr << "Error opening g2o file: " << std::endl;
        // else
        //     std::cerr << "OK opening g2o file: "  << std::endl;
        file << "VERTEX_SE3:QUAT " << std::to_string(pc->header.stamp.toSec()) << " " << position.x << " " << position.y << " " << position.z << " "
             << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() << "\n";
        file.close();

        // 关键帧的个数
        keyframe_cnts++;

        PointCloudXYZRGB scan_world;
        pcl::transformPointCloud(*rgb_pts_cloud, scan_world, transformMatrix_b2w);

        // 放到一起再发出去
        world_rgb_pts += scan_world;
        
        // world_rgb_pts.points.clear();
        // world_rgb_pts = scan_world;

        if (keyframe_cnts % 10 == 0)
        {
            static pcl::VoxelGrid<PointTypeRGB> dsrgb;
            dsrgb.setLeafSize(rgb_map_size, rgb_map_size, rgb_map_size);
            dsrgb.setInputCloud(world_rgb_pts.makeShared());
            dsrgb.filter(world_rgb_pts);
            ROS_WARN("world rgb pts size: %ld .", world_rgb_pts.points.size());
        }

        sensor_msgs::PointCloud2 wpts;
        // pcl::toROSMsg(world_rgb_pts, wpts);
        pcl::toROSMsg(scan_world, wpts);
        wpts.header = pose_msg->header;
        pubLaserCloudFullRes.publish(wpts); // Append the world-frame point cloud to the output.

    }

    void projector::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        mtx_buffer.lock();
        pose_buffer.push_back(pose_msg);
        mtx_buffer.unlock();
    }

 
    
    void projector::matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform & trans)
    {
        
    }

    projector::projector() 
    {
        ros::NodeHandle nh ;
        initParams(nh);

        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, i_params.camera_topic, 10000);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, i_params.pts_topic, 10000);

        subPose = nh.subscribe( i_params.pose_topic , 10000, &projector::pose_callback, this);

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2>
            MySyncPolicy_pts_img;

        // 创建消息同步器，并将订阅器和回调函数绑定到同步器上
        message_filters::Synchronizer<MySyncPolicy_pts_img> sync_pts_img(MySyncPolicy_pts_img(10000), image_sub, pcl_sub );
        sync_pts_img.registerCallback(boost::bind(&projector::projection_img_pts_callback, this, _1, _2));

        image_transport::ImageTransport imageTransport(nh);
        image_publisher = imageTransport.advertise("/project_pc_image", 10000);
        pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/rgb_pts", 10000);

        server_save_rgbmap = nh.advertiseService("save_rgb_map", &projector::save_rgb_map_srv, this);

        ros::spin();
    }



}

int main(int argc, char **argv){
    ros::init(argc, argv, "project_pc_to_image");
    DetectandTract::projector projector;
    return 0;
}
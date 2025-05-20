#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <unistd.h>
#include <map>
#include <vector>
#include <livox_ros_driver/CustomMsg.h>
#include <Eigen/Core>


#include <Eigen/Dense>

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef PointCloudXYZI::Ptr PointCloudXYZIPTR;

string file_path;
string bag_file;
string lidar_topic;
string pcd_file;

bool is_custom_msg;
double limit_y;
int bag_num = 1;

PointCloudXYZI pcl_normal_all;
PointCloudXYZI pcl_outer_all;

// ! @ 三套逻辑的参数说明
//* scan shadow filter log
const float min_angle_tan_ = 10;
const float max_angle_tan_ = 180 - min_angle_tan_;

//* intensity log
// const int widows_size = 50;
const float intensity_step = 5;
const float outlier_percent = 0.05;

//* 所在平面是否与射线方向平行
const float SearchRadius_ = 0.1; // 法向量估计参数，形成局部平面的点所在范围
const float OrthogonalityTHRESHOLD_ = 5.0; // degree 在90度附近的差异范围



bool isShadow(float r1, float r2, float included_angle_sin, float included_angle_cos)
{
  const float perpendicular_y_ = r2 * included_angle_sin;
  const float perpendicular_x_ = r1 - r2 * included_angle_cos;
  const float perpendicular_tan_ = fabs(perpendicular_y_) / perpendicular_x_ * 180 / M_PI;

  if (perpendicular_tan_ > 0)
  {
    if (perpendicular_tan_ < min_angle_tan_)
      return true;
  }
  else
  {
    if (perpendicular_tan_ > max_angle_tan_)
      return true;
  }
  return false;
}

bool isShadow(const float r1, const float r2, const float included_angle)
{
  float included_angle_sin = sinf(included_angle);
  float included_angle_cos = cosf(included_angle);
  return isShadow(r1, r2, included_angle_sin, included_angle_cos);
}


void remove_bleed_points()
{
  PointCloudXYZIPTR cloud_normal_{new PointCloudXYZI()}; 
  PointCloudXYZIPTR cloud_outlier_{new PointCloudXYZI()};

  std::cout << " pcl_normal_all: " << " : " << pcl_normal_all.points.size() << std::endl;
  pcl::PassThrough<PointType> pass_2;
  pass_2.setInputCloud(pcl_normal_all.makeShared());
  pass_2.setFilterFieldName("intensity");
  pass_2.setNegative(true);
  // 把强度为 0 的点去除
  pass_2.setFilterLimits(-5, 0.5);
  pass_2.filter(pcl_normal_all);
  std::cout << " pcl_normal_all:<after remove pts that intensity is 0> " << " : " << pcl_normal_all.points.size() << std::endl;

  // 点云过多的话，后续法向量估计很慢，
  // 先分割出近处点云密度很大的区域，用5cm分辨率去滤波
  // 把远处密度较低的区域全部保留下来
  pcl::CropBox< PointType > cropBoxFilter_temp;
  const double range = 50.0;
  cropBoxFilter_temp.setMin(Eigen::Vector4f(-range, -range, -range, 1.0f));
  cropBoxFilter_temp.setMax(Eigen::Vector4f(range, range, range, 1.0f));
  cropBoxFilter_temp.setInputCloud(pcl_normal_all.makeShared());

  // 之外的点很稀疏，不参与滤波
  PointCloudXYZIPTR cloud_out{new PointCloudXYZI()};
  cropBoxFilter_temp.setNegative(true); // 保留 range 之 外 的 点
  cropBoxFilter_temp.filter(*cloud_out);
  // pcl::io::savePCDFile("/home/cloud_out.pcd", *cloud_out);
  
  cropBoxFilter_temp.setNegative(false); // 保留 range 之内的 点 然后滤波
  cropBoxFilter_temp.filter(pcl_normal_all);

  pcl::VoxelGrid<PointType> downSizeFilterTempMap;
  const double voxel_size = 0.05;
  downSizeFilterTempMap.setLeafSize(voxel_size, voxel_size, voxel_size);
  downSizeFilterTempMap.setInputCloud(pcl_normal_all.makeShared());
  downSizeFilterTempMap.filter(pcl_normal_all);
  std::cout << "pts:< voxel 5cm range within 50m > " << pcl_normal_all.size() << std::endl;

  pcl_normal_all += *cloud_out;
  std::cout << "pts:<beyond 50m pts and voxel 5cm range within 50m > " << pcl_normal_all.size() << std::endl;

  std::cout << "pcl::NormalEstimation with SearchRadius_ : "  << SearchRadius_  << std::endl;
  // 估计法线
  /*        */
  pcl::NormalEstimation<PointType, pcl::Normal> ne;
  ne.setInputCloud( pcl_normal_all.makeShared() ) ;
  // 创建一个空的kdtree对象，并把它传递给法线估计对象
  // 基于给出的输入数据集，kdtree将被建立
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  ne.setSearchMethod(tree);
  // 输出数据集
  pcl::PointCloud<pcl::Normal> cloud_normals;
  // 使用半径在查询点周围3厘米范围内的所有邻元素
  // ne.setRadiusSearch(0.1);
  ne.setRadiusSearch(SearchRadius_);
  // ne.setKSearch(20);

  // 计算特征值
  ne.compute(cloud_normals);

  // 计算每个点的法向量与射线方向的夹角
  for (size_t i = 0; i < cloud_normals.size(); i++)
  {
    PointType pti = pcl_normal_all.points[i];
    pcl::Normal pt = cloud_normals.points[i];

    Eigen::Vector3f this_pt_(pti.x, pti.y, pti.z);
    Eigen::Vector3f this_pt_normal(pt.normal_x, pt.normal_y, pt.normal_z);
 
    double cosValNew = std::fabs(this_pt_.dot(this_pt_normal) / (this_pt_.norm() * this_pt_normal.norm())); // 角度cos值
    double angleNew = std::acos(cosValNew) * 180 / M_PI;                                                    // 角度
    // std::cout << "angleNew: " << angleNew << "  . cosValNew : " << cosValNew << "  . r1 : " << r1 << std::endl;
    if (angleNew < (90.0 + OrthogonalityTHRESHOLD_)  && angleNew > (90.0 - OrthogonalityTHRESHOLD_) ) // cos(80) = 0.17 is good
    {
      cloud_outlier_->points.emplace_back(pti);
    }
    else
    {
      cloud_normal_->points.emplace_back(pti);
    }
  }

  std::cout << "inflation cloud points size : " << cloud_outlier_->size() << std::endl;
  std::cout << "cloud_normal_ cloud points size : " << cloud_normal_->size() << std::endl;
  cloud_outlier_->width = cloud_outlier_->size();
  cloud_outlier_->height = 1;
  cloud_normal_->width = cloud_normal_->size();
  cloud_normal_->height = 1;

  pcl_normal_all = *cloud_normal_;
  pcl_outer_all += *cloud_outlier_;

}


void filter_points_intensity_percent(PointCloudXYZI & pts )
{
  pcl::PassThrough<PointType> pass_1;
  pass_1.setInputCloud(pts.makeShared());
  pass_1.setFilterFieldName("intensity");
  pass_1.setNegative(false);
  // std::cout << " pts: " << " : " << pts.points.size() << std::endl;

  for (int i = 0; i < 255; i = i + intensity_step)
  {
    PointCloudXYZIPTR cloud_out(new PointCloudXYZI);
    pass_1.setFilterLimits(i, i + intensity_step);
    pass_1.filter(*cloud_out);
    // std::cout << "start PassThrough: " << i << " : " << cloud_out->points.size() << std::endl;
    if (cloud_out->size() > pts.size() * outlier_percent )
    {
      pcl_normal_all += *cloud_out;
    }
    else
    {
      pcl_outer_all += *cloud_out;
    }
  }
}

void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in)
{
  PointCloudXYZIPTR one_frame(new PointCloudXYZI);
  PointCloudXYZI pcl_normal;
  PointCloudXYZI pcl_outer;

  std::vector<PointCloudXYZI> pts_6_line;
  pts_6_line.resize(6);

  std::map<int, int> Mymap_line;

  for (unsigned int i = 0; i < livox_msg_in->point_num; ++i)
  {
    PointType pt;
    pt.x = livox_msg_in->points[i].x;
    pt.y = livox_msg_in->points[i].y;
    pt.z = livox_msg_in->points[i].z;
    pt.intensity = livox_msg_in->points[i].reflectivity;

    Mymap_line[livox_msg_in->points[i].line]++;

    int tag = livox_msg_in->points[i].tag;
    if (((tag & 0x30) == 0x10 || (tag & 0x30) == 0x00))
    {
        pts_6_line[livox_msg_in->points[i].line].points.push_back(pt);
    }
  }

  for (size_t i = 0; i < pts_6_line.size(); i++)
  {
    auto &line = pts_6_line[i];
    std::vector<int> indexs;
    for (size_t i = 1; i < line.size() - 1; i++)
    {
      const auto &pt_last = line.points[i - 1];
      const auto &pt = line.points[i];
      const auto &pt_next = line.points[i + 1];
      const float pt_last_range = std::sqrt(pt_last.x * pt_last.x + pt_last.y * pt_last.y + pt_last.z * pt_last.z);
      const float pt_range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      const float pt_next_range = std::sqrt(pt_next.x * pt_next.x + pt_next.y * pt_next.y + pt_next.z * pt_next.z);

      const float pt_last_angle = std::atan2(pt_last.y, pt_last.x);
      const float pt_angle = std::atan2(pt.y, pt.x);
      const float pt_next_angle = std::atan2(pt_next.y, pt_next.x);

      bool f1 = isShadow(pt_last_range, pt_range, pt_last_angle - pt_angle);
      bool f2 = isShadow(pt_range, pt_next_range, pt_angle - pt_next_angle);

      // if (f1 && f2)
      if (f1 || f2)
      {
        if ( std::fabs( pt_last.intensity - pt.intensity ) > 10  &&  std::fabs( pt_next.intensity-pt.intensity ) > 10 )
        {
          indexs.push_back(i);
        }
      }
    }

    // Create the filtering object
    pcl::ExtractIndices<PointType> extract;
    // 索引
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indexs);
    // Extract the inliers
    // ROS_INFO("line : %d ", line.size());
    // ROS_INFO("indexs : %d ", indexs.size());
    extract.setInputCloud(line.makeShared());
    extract.setIndices(index_ptr);

    extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
    extract.filter(pcl_outer);

    // ROS_INFO(" : %d ", pcl_outer.size());
    pcl_outer_all += pcl_outer;

    extract.setNegative(true); // 如果设为true,可以提取指定index之外的点云
    extract.filter(line);
    // ROS_INFO(" : %d ", line.size());
    // pcl_normal += line;
    // ROS_INFO("filter_points_intensity_percent" );
    filter_points_intensity_percent( line );
  }

}

void save_points_from_bag(const string bag_file, const string pcd_file)
{
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_)
  {
    std::string msg = "Loading the rosbag " + bag_file + " failue";
    ROS_ERROR_STREAM(msg.c_str());
    return;
  }
  ROS_INFO("Loading the rosbag %s", bag_file.c_str());
  rosbag::Bag bag;
  try
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e)
  {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return;
  }
  std::vector<string> lidar_topic_vec;
  lidar_topic_vec.push_back(lidar_topic);
  // lidar_topic_vec.push_back("/hk_camera/image_color");
  // lidar_topic_vec.push_back("/hk_camera_info");
  // lidar_topic_vec.push_back("/livox/lidar");
 
  rosbag::View view(bag, rosbag::TopicQuery(lidar_topic_vec));

  int cnts = 0;
  for (const rosbag::MessageInstance &m : view)
  {
    ROS_WARN("msg cnts is: %d .",  cnts++  );
    if (m.getTopic() == "/livox_pc2")
    {
      sensor_msgs::PointCloud2 livox_cloud;
      livox_cloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(livox_cloud, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, cloud);
      for (uint i = 0; i < cloud.size(); ++i)
      {
        if (cloud.points[i].x > 1.0 and cloud.points[i].x < 150.0 and std::fabs(cloud.points[i].y) < limit_y)
        {
          output_cloud.points.push_back(cloud.points[i]);
        }
      }
    }

    if (m.getTopic() == "/livox/lidar")
    {
      //  --------------------------------------------------------------------
      livox_ros_driver::CustomMsgConstPtr livox_msg_in = m.instantiate<livox_ros_driver::CustomMsg>() ;
      // 把这帧数据传给回调函数
      LivoxMsgCbk(livox_msg_in);
      //  --------------------------------------------------------------------
    }
  }

  bag.close();
  
  ROS_WARN("remove_bleed_points" );
  remove_bleed_points();

  // output_cloud.is_dense = false;
  // output_cloud.width = output_cloud.points.size();
  // output_cloud.height = 1;
  // pcl::io::savePCDFileASCII(pcd_file, output_cloud);
  string msg = "Sucessfully save point cloud to pcd file: " + pcd_file;
  ROS_INFO(msg.c_str());

  ROS_WARN("savePCDFile" );
  pcl::io::savePCDFile(pcd_file, pcl_normal_all);
  pcl::io::savePCDFile("/home/pcl_normal_all.pcd", pcl_normal_all);
  pcl::io::savePCDFile("/home/pcl_outer_all.pcd", pcl_outer_all);
  printf("save /home/pcl_normal_all.pcd and /home/pcl_outer_all.pcd \n" );
  ROS_WARN("done" );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("file_path", file_path, "/home/dlvc_data/cab_hall/dvl_bag/");
  // nh.param<string>("pcd_file", pcd_file, "");
  nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
  nh.param<bool>("is_custom_msg", is_custom_msg, false);
  nh.param<double>("limit_y", limit_y, 10.0);
  nh.param<int>("bag_num", bag_num, 5);

  bag_file = "/home/liunao/Kalibr/v1_20241118/bag_o/2.bag";
  pcd_file = "/opt/csg/slam/navs/1.pcd";

  if (argc != 3)
  {
    ROS_ERROR("Usage: %s <bag_file> <pcd_file>", argv[0]);
    return 1;
  }
  // Use command line arguments instead of hardcoded paths

  bag_file = std::string( argv[1] );
  pcd_file = std::string( argv[2] );

  for (int i = 0; i < 1; i++)
  {
    // string bag_file = file_path + std::to_string(i) + ".bag";
    // string pcd_file = file_path + std::to_string(i) + ".pcd";
    string msg = "bag file: " + bag_file;
    ROS_INFO_STREAM(msg.c_str());
    msg = "pcd file: " + pcd_file;
    ROS_INFO_STREAM(msg.c_str());

    save_points_from_bag(bag_file, pcd_file);

  }

  return 0;
}
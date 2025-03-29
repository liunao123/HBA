#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include <unistd.h>
#include <map>
#include <vector>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>

// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef PointCloudXYZI::Ptr PointCloudXYZIPTR;

PointCloudXYZI pcl_normal_all;
PointCloudXYZI pcl_outer_all;

// ! @ 两套逻辑的参数说明
//* scan shadow filter log
const float min_angle_tan_ = 10;
const float max_angle_tan_ = 180 - min_angle_tan_;

//* intensity log
// const int widows_size = 50;
const float intensity_step = 5;
const float outlier_percent = 0.005;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1;
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

// 在全局变量区域添加map来存储不同tag的点云
std::map<int, PointCloudXYZI> tag_clouds;

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

void filterOneFrame(PointCloudXYZI &cloud)
{
  std::vector<int> indices_to_remove;
  for (size_t i = 0; i < cloud.size(); ++i)
  {
    const auto &pt = cloud.points[i];
    Eigen::Vector3f line_vector(pt.x, pt.y, pt.z); // Vector from origin to the point

    for (size_t j = 0; j < cloud.size(); ++j)
    {
      if (i == j)
        continue;
      const auto &other_pt = cloud.points[j];
      Eigen::Vector3f point_vector(other_pt.x, other_pt.y, other_pt.z);

      // Calculate the perpendicular distance from other_pt to the line
      Eigen::Vector3f cross_product = line_vector.cross(point_vector);
      float distance = cross_product.norm() / line_vector.norm();

      if (distance < 0.05)
      { // 1 cm
        indices_to_remove.push_back(i);
        break;
      }
    }
  }

  // Remove points
  pcl::ExtractIndices<PointType> extract;
  boost::shared_ptr<std::vector<int>> indices_ptr = boost::make_shared<std::vector<int>>(indices_to_remove);
  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(indices_ptr);
  extract.setNegative(true);
  PointCloudXYZI t1;
  extract.filter(t1);
  pcl::io::savePCDFileASCII("/opt/csg/slam/navs/true.pcd", t1);
  extract.setNegative(false);
  PointCloudXYZI t2;
  extract.filter(t2);
  pcl::io::savePCDFileASCII("/opt/csg/slam/navs/false.pcd", t2);
  abort();
}

void filter_points_intensity_percent(PointCloudXYZI & pts )
{
  pcl::PassThrough<PointType> pass_1;
  pass_1.setInputCloud(pts.makeShared());
  pass_1.setFilterFieldName("intensity");
  pass_1.setNegative(false);
  std::cout << " pts: " << " : " << pts.points.size() << std::endl;

  for (int i = 0; i < 255; i = i + intensity_step)
  {
    PointCloudXYZIPTR cloud_out(new PointCloudXYZI);
    pass_1.setFilterLimits(i, i + intensity_step);
    pass_1.filter(*cloud_out);

    std::cout << "start PassThrough: " << i << " : " << cloud_out->points.size() << std::endl;
    // if ( 1 )
    if (cloud_out->size() > pts.size() * outlier_percent )
    {
      pcl_normal_all += *cloud_out;
      // sensor_msgs::PointCloud2 pcl_ros_msg1;
      // pcl::toROSMsg(*cloud_out, pcl_ros_msg1);
      // pcl_ros_msg1.header = livox_msg_in->header;
      // pub_pcl_out1.publish(pcl_ros_msg1);
      // pcl::io::savePCDFile("/opt/csg/slam/navs/less_" + std::to_string( i + intensity_step ) + "_.pcd" , *cloud);
    }
    else
    {
      pcl_outer_all += *cloud_out;
      // std::cout << "start remove: " << i  << " : " << pcl_normal_all.size() << std::endl;
    }
    cloud_out->clear();
  }
}


void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in)
{
  PointCloudXYZIPTR one_frame(new PointCloudXYZI);
  PointCloudXYZI pcl_normal;
  PointCloudXYZI pcl_outer;

  std::vector<PointCloudXYZI> pts_6_line;
  pts_6_line.resize(6);

  std::map<int, int> Mymap_tag;
  std::map<int, int> Mymap_line;

  for (unsigned int i = 0; i < livox_msg_in->point_num; ++i)
  {
    PointType pt;
    pt.x = livox_msg_in->points[i].x;
    pt.y = livox_msg_in->points[i].y;
    pt.z = livox_msg_in->points[i].z;
    pt.intensity = livox_msg_in->points[i].reflectivity;

    // 将点加入对应tag的点云中
    int tag = livox_msg_in->points[i].tag;
    tag_clouds[tag].points.push_back(pt);

    // 原有的处理逻辑继续保留
    Mymap_tag[tag]++;
    Mymap_line[livox_msg_in->points[i].line]++;
    if (((tag & 0x30) == 0x10 || (tag & 0x30) == 0x00))
    // if (tag == 16)
    {
      if (pt.x > 1.0 && pt.x < 50.0 && pt.intensity > 2)
      {
        pts_6_line[livox_msg_in->points[i].line].points.push_back(pt);
        // one_frame->points.push_back(pt);
      }
    }
  }

  // Create the filtering object
  pcl::ExtractIndices<PointType> extract;
  for (size_t i = 0; i < pts_6_line.size(); i++)
  {
    auto &line = pts_6_line[i];
    ROS_INFO(" %d th : %d ", i, line.size());
    // pcl::io::savePCDFileASCII("/opt/csg/slam/navs/one_line.pcd", line);
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

      if (f1 && f2)
      // if (f1 || f2)
      {
        // if ( std::fabs( pt_last.intensity - pt.intensity ) > 10  &&  std::fabs( pt_next.intensity-pt.intensity ) > 10 )
        // if (1)
        {
          indexs.push_back(i);
        }
      }
    }

    // 索引
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indexs);
    // Extract the inliers
    ROS_INFO("line : %d ", line.size());
    ROS_INFO("indexs : %d ", indexs.size());
    extract.setInputCloud(line.makeShared());
    extract.setIndices(index_ptr);

    extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
    extract.filter(pcl_outer);

    ROS_INFO(" : %d ", pcl_outer.size());
    pcl_outer_all += pcl_outer;

    extract.setNegative(true); // 如果设为true,可以提取指定index之外的点云
    extract.filter(line);
    // usleep(100000);
    ROS_INFO(" : %d ", line.size());
    // pcl_normal += line;
    filter_points_intensity_percent( line );

    // indexs.clear();
    // line.clear();
  }

  // filter_points_intensity_percent( pcl_normal );

  static int cnts = 0;
  ROS_INFO("cnts is : %d ", cnts);
  if (cnts++ > 400)
  {
    pcl_normal_all.height = 1;
    pcl_normal_all.width = pcl_normal_all.points.size();

    pcl_outer_all.height = 1;
    pcl_outer_all.width = pcl_outer_all.points.size();
    pcl::io::savePCDFile("/opt/csg/slam/navs/pcl_normal_all.pcd", pcl_normal_all);
    pcl::io::savePCDFile("/opt/csg/slam/navs/pcl_outer_all.pcd", pcl_outer_all);
    abort();
  }

  return;

  // ROS_INFO("  : %d ", pcl_normal.size());
  // pcl::PassThrough<PointType> pass;
  // pass.setInputCloud(pcl_normal.makeShared());
  // pass.setFilterFieldName("intensity");
  // pass.setFilterLimits(145, 155);
  // pass.setNegative(true);
  // pass.filter(pcl_normal);
  // ROS_INFO("  : %d ", pcl_normal.size());

  // sensor_msgs::PointCloud2 pcl_ros_msg;
  // pcl::toROSMsg(pcl_normal, pcl_ros_msg);
  // pcl_ros_msg.header = livox_msg_in->header;
  // pub_pcl_out1.publish(pcl_ros_msg);

  // pcl_normal_all += pcl_normal;

  // pass.setNegative(false);
  // pass.filter(pcl_normal);
  // pcl_outer += pcl_normal;

  // pcl::toROSMsg(pcl_outer, pcl_ros_msg);
  // pcl_ros_msg.header = livox_msg_in->header;
  // pub_pcl_out0.publish(pcl_ros_msg);

  // pcl_outer_all += pcl_outer;

  // pcl_normal.clear();
  // pcl_outer.clear();
  // livox_data.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shadow_filter");
  ros::NodeHandle nh;

  ROS_INFO("start shadow_filter");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 1000, LivoxMsgCbk);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pc2", 1000);
  pub_pcl_out0 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pc2_lasi", 1000);

  ros::spin();
}
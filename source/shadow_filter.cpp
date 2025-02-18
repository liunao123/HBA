#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include <unistd.h>
#include <map>
#include <vector>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

PointCloudXYZI pcl_normal_all;
PointCloudXYZI pcl_outer_all;

// ! @ 两套逻辑的参数说明
//* scan shadow filter log
const float min_angle_tan_ = 10;
const float max_angle_tan_ = 180 - min_angle_tan_;

//* intensity log
const int widows_size = 50;
const float intensity_step = 10;
const float outlier_percent = 0.10;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1;
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

bool isShadow(float r1, float r2, float included_angle_sin, float included_angle_cos)
{
  const float perpendicular_y_ = r2 * included_angle_sin;
  const float perpendicular_x_ = r1 - r2 * included_angle_cos;
  const float perpendicular_tan_ = fabs(perpendicular_y_) / perpendicular_x_  * 180 / M_PI ;

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


void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in)
{
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT)
    return;

  PointCloudXYZI pcl_normal;
  PointCloudXYZI pcl_outer;

  std::vector<PointCloudXYZI> pts_6_line;
  pts_6_line.resize(6);

  std::map<int, int> Mymap_tag;
  std::map<int, int> Mymap_line;

  for (size_t j = 0; j < livox_data.size(); j++)
  {
    auto &livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i)
    {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      pt.intensity = livox_msg->points[i].reflectivity;
      Mymap_tag[livox_msg->points[i].tag]++;
      Mymap_line[livox_msg->points[i].line]++;
      // pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
      if (((livox_msg->points[i].tag & 0x30) == 0x10 || (livox_msg->points[i].tag & 0x30) == 0x00))
      {       
        if ( pt.x > 1.0 )
        {
          pts_6_line[livox_msg->points[i].line].points.push_back(pt);
        }

      }
    }
  }

  // Create the filtering object
  pcl::ExtractIndices< PointType > extract;
  for (size_t i = 0; i < pts_6_line.size(); i++)
  {
    auto &line = pts_6_line[i];
    // ROS_INFO(" %d th : %d ", i, line.size());
    // pcl::io::savePCDFileASCII("/opt/csg/slam/navs/one_line.pcd", line);

    std::vector< int > indexs;
    for (size_t i = 1; i < line.size() - 1  ; i++)
    {
      const auto &pt_last = line.points[i - 1];
      const auto &pt = line.points[i];
      const auto &pt_next = line.points[i + 1];
      const float pt_last_range = std::sqrt( pt_last.x*pt_last.x + pt_last.y*pt_last.y + pt_last.z*pt_last.z );
      const float pt_range = std::sqrt( pt.x*pt.x + pt.y*pt.y + pt.z*pt.z );
      const float pt_next_range = std::sqrt( pt_next.x*pt_next.x + pt_next.y*pt_next.y + pt_next.z*pt_next.z );

      const float pt_last_angle = std::atan2( pt_last.y , pt_last.x );
      const float pt_angle = std::atan2( pt.y , pt.x );
      const float pt_next_angle = std::atan2( pt_next.y , pt_next.x );

      bool f1 = isShadow(pt_last_range, pt_range, pt_last_angle - pt_angle );
      bool f2 = isShadow(pt_range, pt_next_range, pt_angle - pt_next_angle );
      
      // if (f1 && f2)
      if (f1 || f2)
      {
        // if ( std::fabs( pt_last.intensity - pt.intensity ) > 10  &&  std::fabs( pt_next.intensity-pt.intensity ) > 10 )
        if ( 1 )
        {
          indexs.push_back(i);
        }
      }
    }

    //索引
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indexs);

    // Extract the inliers
    extract.setInputCloud( line.makeShared() );
    extract.setIndices(index_ptr);
    extract.setNegative( true );//如果设为true,可以提取指定index之外的点云
    extract.filter( line );
    // ROS_INFO(" %d th : %d ", i, line.size());
    indexs.clear();

    extract.setNegative( false );//如果设为true,可以提取指定index之外的点云
    extract.filter( pcl_outer );

    // continue;

    for (size_t i = 1; i < line.size() - widows_size ; i = i + widows_size)
    {
      std::map<int, int> Mymap_intensity;
      for (size_t j = i; j < i + widows_size; j++)
      {
        int key = int( line.points[j].intensity / intensity_step );
        Mymap_intensity[key]++;
      }

      // std::cout << "----------------------------------------------------" << std::endl;
      for ( auto & item : Mymap_intensity)
      {
        // std::cout << "intensity 1 : " << item.first << " .  count : " << item.second << std::endl;
        // 异常点的比例
        if ( item.second < widows_size * outlier_percent )
        {
          item.second = -1;
        }
        // std::cout << "intensity 2 : " << item.first << " .  count : " << item.second << std::endl;
      }
      // std::cout << "----------------------------------------------------" << std::endl;
      for (size_t j = i; j < i + widows_size; j++)
      {
        int key = int( line.points[j].intensity / intensity_step );
        if ( Mymap_intensity[key]  ==  -1 )
        {
          // line.points[j].x = -1;
          // pcl_outer.points.push_back( line.points[j] ) ; 
        }
        else
        {
          indexs.push_back(j);
        }
      }
    }

    //索引
    boost::shared_ptr<std::vector<int>> index_ptr_new = boost::make_shared<std::vector<int>>(indexs);
    // Extract the inliers
    extract.setInputCloud( line.makeShared() );
    extract.setIndices(index_ptr_new);
    extract.setNegative( false );//如果设为true,可以提取指定index之外的点云
    extract.filter( line );
    // usleep(100000);
    pcl_normal += line;
    // ROS_INFO(" : %d ",  line.size());

    extract.setNegative( true );//如果设为true,可以提取指定index之外的点云
    extract.filter( line );
    // ROS_INFO(" : %d ",  line.size());
    pcl_outer += line;
    
    // ROS_INFO(" %d  %d ",pcl_normal.size(), pcl_outer.size()  );

    line.clear();
  }

    // ROS_INFO("  : %d ", pcl_normal.size());
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(pcl_normal.makeShared());
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(145, 155);
    pass.setNegative(true);
    pass.filter(pcl_normal);
    // ROS_INFO("  : %d ", pcl_normal.size());

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg( pcl_normal , pcl_ros_msg);
    pcl_ros_msg.header = livox_msg_in->header;
    pub_pcl_out1.publish(pcl_ros_msg);

    pcl_normal_all += pcl_normal;


    pass.setNegative(false);
    pass.filter(pcl_normal);
    pcl_outer += pcl_normal;

    pcl::toROSMsg( pcl_outer , pcl_ros_msg);
    pcl_ros_msg.header = livox_msg_in->header;
    pub_pcl_out0.publish(pcl_ros_msg);

    pcl_outer_all += pcl_outer;
    
    static int cnts = 0;
    ROS_INFO("cnts is : %d ", cnts );
    if (cnts++ > 450)
    {
      pcl::io::savePCDFileASCII("/opt/csg/slam/navs/pcl_normal_all.pcd", pcl_normal_all);
      pcl::io::savePCDFileASCII("/opt/csg/slam/navs/pcl_outer_all.pcd", pcl_outer_all);
      abort();
    }
    
    pcl_normal.clear();
    pcl_outer.clear();
    livox_data.clear();

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
// #include "pointcloud_utils.hpp"
#include "common_func.cpp"
#include "common.hpp"
#include "adaptive_octree_voxel_filter.hpp"

namespace fs = std::filesystem;

 
// 从文件名提取时间戳字符串
std::string extractTimestamp(const std::string& filename) {
    size_t pos = filename.find('_');
    if (pos != std::string::npos) {
        size_t end_pos = filename.find('.', pos);
        if (end_pos != std::string::npos) {
            return filename.substr(pos + 1, end_pos - pos - 1);
        }
    }
    return "";
}

int main(int argc, char** argv) {
    std::cout << "Usage:  ---------get all pointclouds from one session-----------" << std::endl;
    std::cout << "Usage: " << argv[0] << " <work_dir>  " << std::endl;
    std::cout << "Usage:  --------------------------------------------------------" << std::endl;
    // if (argc != 2) {
    //     return -1;
    // }
    
    // std::string work_dir = argv[1];
    std::string work_dir = "/media/xf/Elements/id4_1202/raw_data_20251220_1/";
    
    std::string pointclouds_dir = work_dir + "/pointclouds";
    std::string poses_dir = work_dir + "/sparse/vehicle_geo_pose";
    std::string output_dir = work_dir + "/sparse/output";
    
    // 创建输出目录
    fs::create_directories(output_dir);
    
    // 获取所有PCD文件
    std::vector<std::string> pcd_files;
    for (const auto& entry : fs::directory_iterator(pointclouds_dir)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path().filename().string());
        }
    }
    
    // 排序文件以确保顺序处理
    std::sort(pcd_files.begin(), pcd_files.end());
    
    std::cout << "Found " << pcd_files.size() << " PCD files" << std::endl;
    
    int processed_count = 0;
    int error_count = 0;
    
    using FilterT = AdaptiveOctreeVoxelFilter<pcl::PointXYZI>;
    FilterT adaptive_filter(5.0f);
    FilterT::PoseList pose_list;
    std::vector<FilterT::CloudPtr> cloud_list;


    int cut = 0;
    for (const auto& pcd_file : pcd_files) {
        // if (cut++ % 3 != 0 )
        if (cut++ > 30 )
        {
            continue;
        }
        try {
            // 提取时间戳
            std::string timestamp_str = extractTimestamp(pcd_file);
            if (timestamp_str.empty()) {
                std::cerr << "Could not extract timestamp from " << pcd_file << std::endl;
                error_count++;
                continue;
            }
            
            // 构建对应的YAML文件名
            std::string yaml_file = poses_dir + "/" + pcd_file.substr(0, pcd_file.length() - 4) + ".yaml";
            std::cerr << "yaml_file file: " << yaml_file << std::endl;
            // std::cerr << "pcd_file file: " << pcd_file << std::endl;
            
            // 检查YAML文件是否存在
            if (!fs::exists(yaml_file)) {
                std::cerr << "Pose file not found: " << yaml_file << std::endl;
                error_count++;
                continue;
            }
            
            // 读取点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>);
            std::string pcd_path = pointclouds_dir + "/" + pcd_file;
            
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud_o) == -1) {
                std::cerr << "Could not read PCD file: " << pcd_path << std::endl;
                error_count++;
                continue;
            }
                // std::cout << "cloud_o " << cloud_o->size()  << std::endl;
            
            // 过滤距离原点小于3米的点
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& point : cloud_o->points) {
                double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if ( point.z < -4.0 || point.z > 20.0 )
                {
                    continue;
                }
                
                if (distance >= 3.0f) {  // 保留距离大于等于3米的点
                    cloud->points.push_back(point);
                }
            }
            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = true;
            // std::cout << "cloud " << cloud->size()  << std::endl;
            
            // 使用过滤后的点云
            // cloud = filtered_cloud;
            
            // 读取位姿数据

            PoseData pose_data = readPoseFromYaml(yaml_file);
            auto success = writePoseToYaml(pose_data, "./1.yaml");
            exit(0);

            pose_list.push_back(pose_data.transformation_matrix);
            cloud_list.push_back(cloud);
            
            processed_count++;
            if (processed_count % 100 == 0) {
                std::cout << "Processed " << processed_count << " files..." << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing " << pcd_file << ": " << e.what() << std::endl;
            error_count++;
        }
    }
    
    if (!adaptive_filter.setKeyPose(pose_list)) {
        std::cerr << "Failed to set key poses: pose/cloud vector size mismatch." << std::endl;
        return -1;
    }
    if (!adaptive_filter.setKeyPointCloud(cloud_list)) {
        std::cerr << "Failed to build unified cloud from batched poses/clouds." << std::endl;
        return -1;
    }

    auto raw_unified_cloud = adaptive_filter.getRawUnifiedCloud();
    std::cout << "local raw unified cloud " << raw_unified_cloud->size()   << std::endl;

    // 为了保证精度，手动遍历每个点进行高精度变换，而不是使用Affine3d
    // 因为大数值的UTM坐标在变换时容易丢失精度
    // 保存到pcd文件 精度会丢失，保存到las文件精度不会丢失
    // std::cout << "Applying high-precision translation offset..." << std::endl;
    
    std::cout << "Translation completed. Points transformed to utm coordinate system." << std::endl;

    std::vector<std::pair<float, float>> adaptive_voxel_params = {
        {0.0f, 0.025f},   // 有位姿
        {10.0f, 0.05f},   // <10m
        {20.0f, 0.1f},    // <20m
        {30.0f, 0.2f},   // <30m
        {FLT_MAX, 0.5f}   // >=30m
    };
    adaptive_filter.setAdaptiveVoxelParams(adaptive_voxel_params);
    auto merged_cloud = adaptive_filter.executeFiltering();
    std::string merged_filename = output_dir + "/filtered_map.pcd";
    std::cout << "\nSaving merged filtered point cloud..." << std::endl;
    pcl::io::savePCDFileBinary(merged_filename, *merged_cloud);
    std::cout << "Saved to: " << merged_filename << std::endl;

    std::string raw_filename = output_dir + "/raw_unfiltered_map.pcd";
    pcl::io::savePCDFileBinary(raw_filename, *raw_unified_cloud);
    std::cout << "Saved raw unified cloud to: " << raw_filename << std::endl;
    
    std::cout << "\n========================================" << std::endl;

    std::cout << "Processing completed!" << std::endl;
    std::cout << "Successfully processed: " << processed_count << " files" << std::endl;
    std::cout << "Raw points: " << raw_unified_cloud->size() << std::endl;
    std::cout << "Filtered points: " << merged_cloud->size() << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;

    return 0;
}

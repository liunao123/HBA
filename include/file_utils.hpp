#pragma once
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>

namespace fs = std::filesystem;

// 配置参数结构体
struct ConfigParams {
	std::string root_folder;
	std::string output_folder;
	std::string keyframe_folder;
	bool enable_save_ply = true;
	bool enable_save_pcd = true;
	bool enable_save_las = true;
	bool enable_save_kml = true;
	bool enable_save_odom = true;
	bool enable_save_image = true;
	bool enable_save_pointcloud = true;
	bool enable_save_pose = true;
	bool enable_use_mercator_origin = false;
	double city_mercator_origin_east = 0.0;
	double city_mercator_origin_north = 0.0;
	double select_point_lat = 0.0;
	double select_point_lon = 0.0;
	double search_range = 200.0;
	double voxel = 0.10;
};

// 递归复制目录结构（不复制文件内容，只建目录）
inline bool copy_directory_structure(const std::string& src, const std::string& dst) {
	namespace fs = std::filesystem;
	try {
		// 找到src下第一个子目录
		fs::path first_subdir;
		for (const auto& entry : fs::directory_iterator(src)) {
			if (entry.is_directory()) {
				first_subdir = entry.path();
				break;
			}
		}
		if (first_subdir.empty()) {
			std::cerr << "[ERROR] No subdirectory found in src: " << src << std::endl;
			return false;
		}
		// 只复制第一个子目录下的结构到dst
		for (const auto& entry : fs::recursive_directory_iterator(first_subdir)) {
			if (entry.is_directory()) {
				auto rel = fs::relative(entry.path(), first_subdir);
				auto target = fs::path(dst) / rel;
				if (!fs::exists(target)) {
					fs::create_directories(target);
				}
			}
		}
		// std::cout << "[INFO] Directory structure copied from " << first_subdir << " to " << dst << std::endl;

		// 复制 calib 文件夹的全部内容到 dst
		fs::path calib_src = fs::path(first_subdir) / "calib";
		if (fs::exists(calib_src) && fs::is_directory(calib_src)) {
			fs::path calib_dst = fs::path(dst) / "calib";
			
			try {
				// 使用 copy 函数直接复制整个文件夹
				fs::copy(calib_src, calib_dst, 
					fs::copy_options::recursive | fs::copy_options::overwrite_existing);
				// std::cout << "[INFO] Calib folder copied from " << calib_src << " to " << calib_dst << std::endl;
			} catch (const std::exception& e) {
				std::cerr << "[ERROR] Failed to copy calib folder: " << e.what() << std::endl;
			}
		} else {
			std::cout << "[WARNING] Calib folder not found in: " << first_subdir << std::endl;
		}


		return true;
	} catch (const std::exception& e) {
		std::cerr << "[ERROR] Failed to copy directory structure: " << e.what() << std::endl;
		return false;
	}
}


// 读取config.yaml参数
inline ConfigParams readConfigParams(const std::string& yaml_file) {
	ConfigParams params;
	YAML::Node config = YAML::LoadFile(yaml_file);
	if (config["root_folder"]) params.root_folder = config["root_folder"].as<std::string>();
	if (config["output_folder"]) params.output_folder = config["output_folder"].as<std::string>();
	if (config["enable_save_ply"]) params.enable_save_ply = config["enable_save_ply"].as<bool>();
	if (config["enable_save_las"]) params.enable_save_las = config["enable_save_las"].as<bool>();
	if (config["enable_save_pcd"]) params.enable_save_pcd = config["enable_save_pcd"].as<bool>();
	if (config["enable_save_kml"]) params.enable_save_kml = config["enable_save_kml"].as<bool>();
	if (config["enable_save_odom"]) params.enable_save_odom = config["enable_save_odom"].as<bool>();
	if (config["enable_save_image"]) params.enable_save_image = config["enable_save_image"].as<bool>();
	if (config["enable_save_pointcloud"]) params.enable_save_pointcloud = config["enable_save_pointcloud"].as<bool>();
	if (config["enable_save_pose"]) params.enable_save_pose = config["enable_save_pose"].as<bool>();
	if (config["enable_use_mercator_origin"]) params.enable_use_mercator_origin = config["enable_use_mercator_origin"].as<bool>();
	if (config["city_mercator_origin_east"]) params.city_mercator_origin_east = config["city_mercator_origin_east"].as<double>();
	if (config["city_mercator_origin_north"]) params.city_mercator_origin_north = config["city_mercator_origin_north"].as<double>();
	if (config["select_point_lat"]) params.select_point_lat = config["select_point_lat"].as<double>();
	if (config["select_point_lon"]) params.select_point_lon = config["select_point_lon"].as<double>();
	if (config["search_range"]) params.search_range = config["search_range"].as<double>();
	if (config["voxel"]) params.voxel = config["voxel"].as<double>();
	params.keyframe_folder = params.output_folder + "/KeyFrames/";

	std::cout << "----------------------------------------" << std::endl;
	std::cout << "Config parameters loaded:" << std::endl;
	std::cout << "root_folder: " << params.root_folder << std::endl;
	std::cout << "output_folder: " << params.output_folder << std::endl;
	std::cout << "enable_save_ply: " << std::boolalpha << params.enable_save_ply << std::endl;
	std::cout << "enable_save_las: " << std::boolalpha << params.enable_save_las << std::endl;
	std::cout << "enable_save_pcd: " << std::boolalpha << params.enable_save_pcd << std::endl;
	std::cout << "enable_save_kml: " << std::boolalpha << params.enable_save_kml << std::endl;
	std::cout << "enable_save_odom: " << std::boolalpha << params.enable_save_odom << std::endl;
	std::cout << "enable_save_image: " << std::boolalpha << params.enable_save_image << std::endl;
	std::cout << "enable_save_pointcloud: " << std::boolalpha << params.enable_save_pointcloud << std::endl;
	std::cout << "enable_save_pose: " << std::boolalpha << params.enable_save_pose << std::endl;
	std::cout << "enable_use_mercator_origin: " << std::boolalpha << params.enable_use_mercator_origin << std::endl;
	std::cout << "city_mercator_origin_east: " << std::fixed << std::setprecision(2) << params.city_mercator_origin_east << std::endl;
	std::cout << "city_mercator_origin_north: " << std::fixed << std::setprecision(2) << params.city_mercator_origin_north << std::endl;
	std::cout << "select_point_lat: " << params.select_point_lat << std::endl;
	std::cout << "select_point_lon: " << params.select_point_lon << std::endl;
	std::cout << "search_range: " << params.search_range << " m" << std::endl;
	std::cout << "voxel: " << params.voxel << " m" << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	return params;
}


inline std::vector<std::string> getAllFilePaths(const std::string &root_dir, const std::string &extension = "")
{
	std::vector<std::string> file_paths;
	if (!fs::exists(root_dir) || !fs::is_directory(root_dir))
	{
		std::cerr << "Error: Directory does not exist: " << root_dir << std::endl;
		return file_paths;
	}
	try
	{
		std::vector<std::string> target_subdirs;
		target_subdirs.push_back("pointclouds/");
		target_subdirs.push_back("sparse/vehicle_geo_pose"); //! 必须要有这个子目录
		for (const auto &middle_entry : fs::directory_iterator(root_dir))
		{
			if (middle_entry.is_directory())
			{
				std::string middle_dir_path = middle_entry.path().string();
				for (const std::string &subdir : target_subdirs)
				{
					std::string full_subdir_path = middle_dir_path + "/" + subdir;
					// std::cout << "Searching : Subdirectory : " << full_subdir_path << std::endl;

					if (!fs::exists(full_subdir_path) || !fs::is_directory(full_subdir_path))
					{
						std::cout << "Warning: Subdirectory does not exist: " << full_subdir_path << std::endl;
						continue;
					}
					int cnt = 0;
					for (const auto &file_entry : fs::directory_iterator(full_subdir_path))
					{
						if (file_entry.is_regular_file())
						{
							if (extension.empty() || file_entry.path().extension() == extension)
							{
								file_paths.push_back(file_entry.path().string());
								cnt++;
							}
						}
					}
					// std::cout << "Finding file number: " << cnt << std::endl;
				}
			}
		}
	}
	catch (const fs::filesystem_error &ex)
	{
		std::cerr << "Filesystem error: " << ex.what() << std::endl;
	}
	catch (const std::exception &ex)
	{
		std::cerr << "Error: " << ex.what() << std::endl;
	}
	return file_paths;
}

// Convert YAML file paths to corresponding PCD file paths
// Replaces directory and extension: /path/to/odoms/1.yaml -> /path/to/pointclouds/1.pcd
inline std::vector<std::string> convertYamlPathsToPcdPaths(
	const std::vector<std::string>& yaml_files,
	const std::string& pcd_directory)
{
	std::vector<std::string> pcd_paths;
	pcd_paths.reserve(yaml_files.size());
	
	for (const auto& yaml_file : yaml_files) {
		// Get the filename without extension
		fs::path yaml_path(yaml_file);
		std::string stem = yaml_path.stem().string();  // filename without extension
		
		// Construct PCD path: pcd_directory + stem + ".pcd"
		fs::path pcd_path = fs::path(pcd_directory) / (stem + ".pcd");
		pcd_paths.push_back(pcd_path.string());
	}
	
	std::cout << "Converted " << yaml_files.size() << " YAML paths to PCD paths" << std::endl;
	std::cout << "PCD directory: " << pcd_directory << std::endl;
	
	if (!pcd_paths.empty()) {
		std::cout << "Example: " << yaml_files[0] << " -> " << pcd_paths[0] << std::endl;
	}
	
	return pcd_paths;
}

inline double extractTimestampFromFileName(const std::string &filename)
{
	size_t underscore_pos = filename.find('_');
	size_t dot_pos = filename.find_last_of('.');
	if (underscore_pos != std::string::npos && dot_pos != std::string::npos && underscore_pos < dot_pos)
	{
		std::string timestamp_str = filename.substr(underscore_pos + 1, dot_pos - underscore_pos - 1);
		return std::stod(timestamp_str);
	}
	return -1.0;
}

inline std::string findCorrespondingPcdFile(const std::string &pose_file_path, const std::vector<std::string> &all_pcd_files)
{
	std::string pose_filename = fs::path(pose_file_path).filename().string();
	double pose_timestamp = extractTimestampFromFileName(pose_filename);
	if (pose_timestamp < 0)
	{
		std::cerr << "Could not extract timestamp from pose file: " << pose_filename << std::endl;
		return "";
	}
	for (const std::string &pcd_file : all_pcd_files)
	{
		std::string pcd_filename = fs::path(pcd_file).filename().string();
		double pcd_timestamp = extractTimestampFromFileName(pcd_filename);
		if (std::abs(pose_timestamp - pcd_timestamp) < 0.001)
		{
			return pcd_file;
		}
	}
	std::cerr << "No corresponding PCD file found for pose file: " << pose_filename << std::endl;
	return "";
}

inline std::string constructPcdFileName(const std::string &root_dir, int pose_id, double timestamp)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(3) << timestamp;
	std::string timestamp_str = oss.str();
	std::string filename = std::to_string(pose_id) + "_" + timestamp_str + ".pcd";
	return root_dir + "/pointclouds/" + filename;
}



// 只传pose_file_path，自动提取ros2_root_dir（sparse之前的路径）
inline std::vector<std::string> findRelatedFiles(const std::string& pose_file_path)
{
	// 1. 提取ros2_root_dir
	std::string ros2_root_dir;
	size_t sparse_pos = pose_file_path.find("sparse");
	if (sparse_pos != std::string::npos) {
		ros2_root_dir = pose_file_path.substr(0, sparse_pos);
		// 去除末尾的斜杠
		if (!ros2_root_dir.empty() && (ros2_root_dir.back() == '/' || ros2_root_dir.back() == '\\'))
			ros2_root_dir.pop_back();
	} else {
		// fallback: 用父目录
		ros2_root_dir = std::filesystem::path(pose_file_path).parent_path().parent_path().string();
	}

	// std::cout << "pose_file_path: " << pose_file_path << std::endl;
	// std::cout << "ros2_root_dir: " << ros2_root_dir << std::endl;

	// 2. 提取id和timestamp
	std::string filename = std::filesystem::path(pose_file_path).filename().string();
	size_t underscore = filename.find('_');
	size_t dot = filename.find_last_of('.');
	if (underscore == std::string::npos || dot == std::string::npos || underscore >= dot) {
		return {};
	}
	std::string id = filename.substr(0, underscore);
	std::string timestamp = filename.substr(underscore + 1, dot - underscore - 1);

	// 构造各类文件名
	std::ostringstream ts_stream;
	ts_stream << std::fixed << std::setprecision(3) << std::stod(timestamp);
	std::string ts_str = ts_stream.str();

	std::vector<std::string> result;
	// images: cam1 to cam7
	for (int cam = 1; cam <= 7; ++cam) {
		std::ostringstream cam_dir;
		cam_dir << ros2_root_dir << "/images/cam" << cam << "/" << id << "_" << ts_str << "_cam" << cam << ".jpg";
		std::string image_file = cam_dir.str();
		// std::cout << "image_file: " << image_file << std::endl;
		if (std::filesystem::exists(image_file))
			result.push_back(image_file);
		else
			result.push_back("");
	}

	// odoms
	std::string odom_file = ros2_root_dir + "/odoms/" + id + "_" + ts_str + ".yaml";
	// std::cout << "odom_file: " << odom_file << std::endl;
	if (std::filesystem::exists(odom_file)) result.push_back(odom_file); else result.push_back("");
	
	// pointclouds
	std::string pcd_file = ros2_root_dir + "/pointclouds/" + id + "_" + ts_str + ".pcd";
	// std::cout << "pcd_file: " << pcd_file << std::endl;
	if (std::filesystem::exists(pcd_file)) result.push_back(pcd_file); else result.push_back("");

	return result;
}

// 复制文件到输出目录，保持相对路径结构
inline bool copyFileWithStructure(const std::string& src_file, const std::string& src_root, const std::string& dst_root) {
	namespace fs = std::filesystem;
	
	if (src_file.empty() || !fs::exists(src_file)) {
		return false;
	}
	
	try {
		// 计算相对路径
		fs::path src_path(src_file);
		fs::path src_root_path(src_root);
		fs::path relative = fs::relative(src_path, src_root_path);
		
		// 构建目标路径
		fs::path dst_path = fs::path(dst_root) / relative;
		
		// 创建目标目录
		fs::create_directories(dst_path.parent_path());
		
		// 复制文件（如果目标文件不存在）
		if (!fs::exists(dst_path)) {
			fs::copy_file(src_path, dst_path, fs::copy_options::overwrite_existing);
			// std::cout << "Copied: " << src_file << " -> " << dst_path << std::endl;
		}
		
		return true;
	} catch (const std::exception& e) {
		std::cerr << "Error copying file " << src_file << ": " << e.what() << std::endl;
		return false;
	}
}

// 判断文件是否应该被复制（根据配置和文件类型）
inline bool shouldCopyFile(const std::string& file_path, const ConfigParams& params) {
	if (file_path.empty() || !std::filesystem::exists(file_path)) {
		return false;
	}
	
	std::string ext = std::filesystem::path(file_path).extension().string();
	
	// 图像文件 (.jpg) - 需要是 .jpg 文件 且 在 /images/ 目录 且 配置允许
	if (ext == ".jpg" && file_path.find("/images/") != std::string::npos && params.enable_save_image) {
		return true;
	}
	
	// odom 文件 (.yaml in odoms/) - 需要是 .yaml 文件 且 在 /odoms/ 目录 且 配置允许
	if (ext == ".yaml" && file_path.find("/odoms/") != std::string::npos && params.enable_save_odom) {
		return true;
	}
	
	// pose 文件 (.yaml in sparse/) - 需要是 .yaml 文件 且 在 /sparse/ 目录 且 配置允许
	if (ext == ".yaml" && file_path.find("/sparse/") != std::string::npos && params.enable_save_pose) {
		return true;
	}
	
	// 点云文件 (.pcd) - 需要是 .pcd 文件 且 在 /pointclouds/ 目录 且 配置允许
	if (ext == ".pcd" && file_path.find("/pointclouds/") != std::string::npos && params.enable_save_pointcloud) {
		return true;
	}
	
	return false;
}

// 批量复制相关文件
inline void copyRelatedFiles(const std::vector<std::string>& absolut_files, 
                               const std::string& dst_root,
                               const ConfigParams& params) {
	// 注意：absolut_files 中的路径是绝对路径
	// 我们需要提取其相对于某个数据包根目录的相对路径
	
	for (const auto& file : absolut_files) {
		// 根据配置判断是否应该复制此文件
		if (!shouldCopyFile(file, params)) {
			continue;
		}
		
		// 查找文件路径中的数据包根目录（包含 images/odoms/pointclouds/sparse 的目录）
		// 例如：/path/to/map_result/bag_001/images/cam1/xxx.jpg
		// 我们需要找到 bag_001 所在的位置作为相对路径的起点
		
		std::string file_root;
		size_t pos = std::string::npos;
		
		// 尝试找到 images/, odoms/, pointclouds/, 或 sparse/ 之前的部分
		if (file.find("/images/") != std::string::npos) {
			pos = file.find("/images/");
		} else if (file.find("/odoms/") != std::string::npos) {
			pos = file.find("/odoms/");
		} else if (file.find("/pointclouds/") != std::string::npos) {
			pos = file.find("/pointclouds/");
		}
		 else if (file.find("/sparse/") != std::string::npos) {
			pos = file.find("/sparse/");
		}
		
		if (pos != std::string::npos) {
			file_root = file.substr(0, pos);
			copyFileWithStructure(file, file_root, dst_root);
		}
	}
}

// Get all files with specified extension from a directory
// Returns a vector of absolute file paths
inline std::vector<std::string> getFilesWithExtension(const std::string &directory, const std::string &extension = ".yaml")
{
	std::vector<std::string> file_paths;
	
	if (!fs::exists(directory)) {
		std::cerr << "Error: Directory does not exist: " << directory << std::endl;
		return file_paths;
	}
	
	if (!fs::is_directory(directory)) {
		std::cerr << "Error: Path is not a directory: " << directory << std::endl;
		return file_paths;
	}
	
	try {
		for (const auto &entry : fs::directory_iterator(directory)) {
			if (entry.is_regular_file()) {
				std::string file_path = entry.path().string();
				std::string file_ext = entry.path().extension().string();
				
				// Check if extension matches (case-insensitive comparison)
				if (file_ext == extension || 
					(extension[0] != '.' && file_ext == "." + extension)) {
					file_paths.push_back(fs::absolute(entry.path()).string());
				}
			}
		}
		
		// Natural sort by filename (numbers are compared numerically: 1, 2, 10, 11...)
		std::sort(file_paths.begin(), file_paths.end(), 
			[](const std::string& a, const std::string& b) {
				std::string name_a = fs::path(a).filename().string();
				std::string name_b = fs::path(b).filename().string();
				
				size_t i = 0, j = 0;
				while (i < name_a.length() && j < name_b.length()) {
					// If both characters are digits, compare numerically
					if (std::isdigit(name_a[i]) && std::isdigit(name_b[j])) {
						// Extract the full number
						size_t num_a = 0, num_b = 0;
						while (i < name_a.length() && std::isdigit(name_a[i])) {
							num_a = num_a * 10 + (name_a[i] - '0');
							i++;
						}
						while (j < name_b.length() && std::isdigit(name_b[j])) {
							num_b = num_b * 10 + (name_b[j] - '0');
							j++;
						}
						if (num_a != num_b) return num_a < num_b;
					} else {
						// Compare characters normally
						if (name_a[i] != name_b[j]) return name_a[i] < name_b[j];
						i++;
						j++;
					}
				}
				return name_a.length() < name_b.length();
			});
		
		std::cout << "Found " << file_paths.size() << " files with extension '" 
		          << extension << "' in directory: " << directory << std::endl;
		
	} catch (const fs::filesystem_error &ex) {
		std::cerr << "Filesystem error: " << ex.what() << std::endl;
	} catch (const std::exception &ex) {
		std::cerr << "Error: " << ex.what() << std::endl;
	}
	
	return file_paths;
}

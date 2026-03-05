#ifndef COMMON_FUNC_CPP
#define COMMON_FUNC_CPP
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sys/stat.h>
#include <sys/types.h>

#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <map>

namespace fs = std::filesystem;


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



void create_dir_if_not_exists(const std::string& dir) 
{
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::cerr << "Failed to create directory " << dir << ": " << ec.message() << std::endl;
    }
}

// 1. 输入参数合法性检查
std::vector<std::string> collect_frame_timestamps(const std::string &pcd_root)
{
    std::vector<std::string> frames;
    try
    {
        for (const auto &entry : std::filesystem::directory_iterator(pcd_root))
        {
            if (!entry.is_regular_file())
                continue;
            const auto &path = entry.path();
            if (path.extension() != ".pcd")
                continue;
            frames.push_back(path.stem().string());
        }
        std::sort(frames.begin(), frames.end());
    }
    catch (const std::exception &e)
    {
        std::cerr << "[collect_frame_timestamps] Error reading " << pcd_root << ": " << e.what() << std::endl;
    }
    return frames;
}


#endif // COMMON_FUNC_CPP

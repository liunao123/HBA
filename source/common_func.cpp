#ifndef COMMON_FUNC_CPP
#define COMMON_FUNC_CPP

#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <sys/stat.h>
#include <sys/types.h>

void create_dir_if_not_exists(const std::string& dir) 
{
  struct stat st;
  if (stat(dir.c_str(), &st) != 0) {
    mkdir(dir.c_str(), 0777);
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

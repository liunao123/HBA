/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, HBA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * @file   ExtrinsicManagerDemo.cpp
 * @brief  TF Cache system (ROS-independent core + ROS wrapper)
 * 
 * This file contains:
 *   1. ExtrinsicManager - ROS-independent TF tree management
 *   2. ExtrinsicManagerNode - ROS node wrapper for publishing static TFs
 * 
 * @author HBA Team
 * @date   2026-03-02
 */

#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// ROS dependencies (only for wrapper)
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// ============================================================================
// ROS-Independent Core Classes
// ============================================================================

/**
 * @brief TFTransform: 4x4 matrix wrapper
 */
struct TFTransform
{
    Eigen::Matrix4d mat;
    TFTransform() : mat(Eigen::Matrix4d::Identity()) {}
    TFTransform(const Eigen::Matrix4d &m) : mat(m) {}
};

/**
 * @brief ExtrinsicManager: Manage TF tree and lookup transformations
 * 
 * This class manages a transformation tree similar to tf2/tf in ROS,
 * but without any ROS dependencies. Can be used in any C++ project.
 */
class ExtrinsicManager
{
public:
    /**
     * @brief Lookup transform from source to target frame
     * 
     * Implements interface consistent with tf2_ros Buffer::lookupTransform
     * 
     * @param target_frame Target coordinate frame
     * @param source_frame Source coordinate frame
     * @param out Output transformation matrix (source in target frame)
     * @return True if transform found
     */
    bool lookupTransform(const std::string &target_frame, 
                        const std::string &source_frame, 
                        Eigen::Matrix4d &out)
    {
        if (target_frame == source_frame)
        {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        // Track visited nodes to prevent cycles
        std::unordered_set<std::string> visited;
        return lookupTransformImpl(target_frame, source_frame, out, visited);
    }

    /**
     * @brief Add a transformation between frames
     * 
     * @param parent Parent frame name
     * @param child Child frame name
     * @param tf Transformation matrix (parent -> child)
     */
    void addTF(const std::string &parent, const std::string &child, const Eigen::Matrix4d &tf)
    {
        tf_map_[child].emplace_back(parent, TFTransform(tf));
        std::cout << "[ExtrinsicManager] Added TF: " << parent << " -> " << child << std::endl;
    }

    /**
     * @brief Load all extrinsic files from folder
     * 
     * @param folder Path to folder containing YAML extrinsic files
     */
    void loadFolder(const std::string &folder)
    {
        std::cout << "[ExtrinsicManager] Loading TF files from: " << folder << std::endl;
        int count = 0;
        for (const auto &entry : std::filesystem::directory_iterator(folder))
        {
            if (entry.is_regular_file())
            {
                loadFile(entry.path().string());
                count++;
            }
        }
        std::cout << "[ExtrinsicManager] Loaded " << count << " TF files" << std::endl;
    }

    /**
     * @brief Get all frame names in the TF tree
     * 
     * @return Vector of frame names
     */
    std::vector<std::string> getAllFrames() const
    {
        std::unordered_set<std::string> frames;
        for (const auto &[child, vec] : tf_map_)
        {
            frames.insert(child);
            for (const auto &[parent, tf] : vec)
            {
                frames.insert(parent);
            }
        }
        return std::vector<std::string>(frames.begin(), frames.end());
    }

    /**
     * @brief Get transformation map (for ROS wrapper access)
     * 
     * @return Reference to internal TF map
     */
    const std::unordered_map<std::string, std::vector<std::pair<std::string, TFTransform>>>& 
    getTFMap() const
    {
        return tf_map_;
    }

    /**
     * @brief Print TF tree structure
     */
    void printTree() const
    {
        std::cout << "\n[ExtrinsicManager] TF Tree Structure:" << std::endl;
        std::cout << "======================================" << std::endl;
        for (const auto &[child, vec] : tf_map_)
        {
            for (const auto &[parent, tf] : vec)
            {
                std::cout << parent << " --> " << child << std::endl;
            }
        }
        std::cout << "======================================\n" << std::endl;
    }

private:
    /**
     * @brief Recursive implementation of lookupTransform with cycle detection
     */
    bool lookupTransformImpl(const std::string &target, 
                            const std::string &source, 
                            Eigen::Matrix4d &out, 
                            std::unordered_set<std::string> &visited)
    {
        if (target == source)
        {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        if (visited.count(source))
            return false;
        visited.insert(source);
        
        // Search through parent nodes
        if (tf_map_.count(source))
        {
            for (const auto &[parent, tf] : tf_map_[source])
            {
                Eigen::Matrix4d parent2target;
                if (lookupTransformImpl(target, parent, parent2target, visited))
                {
                    out = parent2target * tf.mat;
                    return true;
                }
            }
        }
        
        // Search through child nodes (inverse direction)
        for (const auto &[maybe_child, vec] : tf_map_)
        {
            for (const auto &[parent, tf] : vec)
            {
                if (parent == source)
                {
                    Eigen::Matrix4d child2target;
                    if (lookupTransformImpl(target, maybe_child, child2target, visited))
                    {
                        out = child2target * tf.mat.inverse();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * @brief Load one extrinsic file (YAML format)
     * 
     * Format: parent child 4x4 matrix (quaternion + translation)
     */
    void loadFile(const std::string &file)
    {
        // Extract parent and child from filename
        std::string filename = file.substr(file.find_last_of("/\\") + 1);
        std::vector<std::string> tokens;
        std::stringstream ss(filename);
        std::string token;
        while (std::getline(ss, token, '_'))
            tokens.push_back(token);
        
        std::string parent, child;
        if (tokens.size() >= 4)
        {
            parent = tokens[3];
            child = tokens[1];
            size_t dot = child.find('.');
            if (dot != std::string::npos)
                child = child.substr(0, dot);
        }
        
        // Parse YAML file
        try
        {
            YAML::Node root = YAML::LoadFile(file);
            std::vector<double> quat(4, 0.0);
            std::vector<double> trans(3, 0.0);
            
            if (root["r_quaternion_wxyz"])
            {
                auto qdata = root["r_quaternion_wxyz"]["data"];
                for (int i = 0; i < 4; ++i)
                    quat[i] = qdata[i].as<double>();
            }
            if (root["t_metric_xyz"])
            {
                auto tdata = root["t_metric_xyz"]["data"];
                for (int i = 0; i < 3; ++i)
                    trans[i] = tdata[i].as<double>();
            }
            
            if (!parent.empty() && !child.empty())
            {
                Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
                Eigen::Matrix3d R = q.normalized().toRotationMatrix();
                Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
                mat.block<3, 3>(0, 0) = R;
                mat(0, 3) = trans[0];
                mat(1, 3) = trans[1];
                mat(2, 3) = trans[2];
                addTF(parent, child, mat);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ExtrinsicManager] YAML parse error in " << file << ": " << e.what() << std::endl;
        }
    }

    // Child -> vector of (parent, transform)
    std::unordered_map<std::string, std::vector<std::pair<std::string, TFTransform>>> tf_map_;
};

// ============================================================================
// Camera Intrinsics Loader (ROS-independent)
// ============================================================================

/**
 * @brief CameraIntrinsic: Load and query camera intrinsics from YAML files
 *
 * This class loads all YAML files in a folder and stores per-camera
 * intrinsic parameters in OpenCV format (K and D matrices).
 *
 * Camera name is extracted from the filename (stem without extension).
 */
class CameraIntrinsic
{
public:
    struct Intrinsic
    {
        cv::Mat K;   // 3x3 camera matrix
        cv::Mat D;   // distortion coefficients (Nx1)
        int width = 0;
        int height = 0;
    };

    /**
     * @brief Load all intrinsic YAML files from folder
     *
     * @param folder Path to folder containing YAML intrinsics
     */
    void loadFolder(const std::string &folder)
    {
        std::cout << "[CameraIntrinsic] Loading intrinsics from: " << folder << std::endl;
        int count = 0;
        for (const auto &entry : std::filesystem::directory_iterator(folder))
        {
            if (entry.is_regular_file())
            {
                loadFile(entry.path().string());
                count++;
            }
        }
        std::cout << "[CameraIntrinsic] Loaded " << count << " intrinsics files" << std::endl;
    }

    /**
     * @brief Query intrinsics by camera name
     *
     * @param camera_name Camera name (from filename stem)
     * @param K Output 3x3 camera matrix
     * @param D Output distortion coefficients
     * @param width Optional output image width
     * @param height Optional output image height
     * @return True if found
     */
    bool getIntrinsic(const std::string &camera_name,
                      cv::Mat &K,
                      cv::Mat &D,
                      int *width = nullptr,
                      int *height = nullptr) const
    {
        auto it = intrinsics_.find(camera_name);
        if (it == intrinsics_.end())
            return false;
        K = it->second.K.clone();
        D = it->second.D.clone();
        if (width)
            *width = it->second.width;
        if (height)
            *height = it->second.height;
        return true;
    }

    /**
     * @brief Get all camera names
     */
    std::vector<std::string> getAllCameraNames() const
    {
        std::vector<std::string> names;
        names.reserve(intrinsics_.size());
        for (const auto &kv : intrinsics_)
            names.push_back(kv.first);
        return names;
    }

private:
    static std::vector<double> readVector(const YAML::Node &node)
    {
        std::vector<double> data;
        if (!node || !node.IsSequence())
            return data;
        data.reserve(node.size());
        for (size_t i = 0; i < node.size(); ++i)
            data.push_back(node[i].as<double>());
        return data;
    }

    static bool extractMatrixData(const YAML::Node &root,
                                  const std::string &key,
                                  std::vector<double> &out)
    {
        if (root[key])
        {
            if (root[key]["data"])
            {
                out = readVector(root[key]["data"]);
                return !out.empty();
            }
            if (root[key].IsSequence())
            {
                out = readVector(root[key]);
                return !out.empty();
            }
        }
        return false;
    }

    void loadFile(const std::string &file)
    {
        std::string filename = file.substr(file.find_last_of("/\\") + 1);
        
        // Extract camera name (e.g., "cam5" from "cam5_intrinsics.yaml")
        std::string camera_name = std::filesystem::path(filename).stem().string().substr(0, 4); 

        try
        {
            YAML::Node root = YAML::LoadFile(file);

            std::vector<double> kdata;
            std::vector<double> ddata;

            // Common keys: cameraMatrix/camera_matrix, distCoeffs/distortion_coefficients
            bool has_k = extractMatrixData(root, "cameraMatrix", kdata);
            bool has_d = extractMatrixData(root, "distCoeffs", ddata);

            if (!has_k)
                has_k = extractMatrixData(root, "camera_matrix", kdata);
            if (!has_d)
                has_d = extractMatrixData(root, "distortion_coefficients", ddata);

            // Fallback keys: K, D
            if (!has_k)
                has_k = extractMatrixData(root, "K", kdata);
            if (!has_d)
                has_d = extractMatrixData(root, "D", ddata);

            if (!has_k || kdata.size() < 9)
            {
                std::cerr << "[CameraIntrinsic] Missing camera_matrix in " << file << std::endl;
                return;
            }

            Intrinsic intr;
            intr.K = cv::Mat(3, 3, CV_64F);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    intr.K.at<double>(r, c) = kdata[r * 3 + c];

            if (!ddata.empty())
            {
                intr.D = cv::Mat(static_cast<int>(ddata.size()), 1, CV_64F);
                for (size_t i = 0; i < ddata.size(); ++i)
                    intr.D.at<double>(static_cast<int>(i), 0) = ddata[i];
            }
            else
            {
                intr.D = cv::Mat::zeros(0, 1, CV_64F);
            }

            if (root["ImageWidth"])
                intr.width = root["ImageWidth"].as<int>();
            else if (root["image_width"])
                intr.width = root["image_width"].as<int>();
            if (root["ImageHeight"])
                intr.height = root["ImageHeight"].as<int>();
            else if (root["image_height"])
                intr.height = root["image_height"].as<int>();

            intrinsics_[camera_name] = intr;
            std::cout << "[CameraIntrinsic] Loaded: " << camera_name << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[CameraIntrinsic] YAML parse error in " << file << ": " << e.what() << std::endl;
        }
    }

    std::unordered_map<std::string, Intrinsic> intrinsics_;
};

// ============================================================================
// ROS Node Wrapper
// ROS Node Wrapper
// ============================================================================

/**
 * @brief ExtrinsicManagerNode: ROS wrapper for publishing static transforms
 * 
 * Wraps ExtrinsicManager to publish static transforms via tf2_ros
 */
class ExtrinsicManagerNode
{
public:
    /**
     * @brief Constructor
     * 
     * @param nh ROS node handle
     */
    explicit ExtrinsicManagerNode(ros::NodeHandle &nh) : nh_(nh)
    {
        std::cout << "[ExtrinsicManagerNode] Initialized" << std::endl;
    }

    /**
     * @brief Get reference to internal ExtrinsicManager
     * 
     * @return Reference to ExtrinsicManager instance
     */
    ExtrinsicManager& getExtrinsicManager()
    {
        return tf_cache_;
    }

    /**
     * @brief Publish all static TFs to ROS
     * 
     * Broadcasts all transformations in the TF tree as static transforms
     * 
     * @param rate_hz Publishing rate (default 1 Hz)
     */
    void publishAllStaticTFROS(double rate_hz = 1.0)
    {
        std::cout << "[ExtrinsicManagerNode] Starting static TF broadcaster at " << rate_hz << " Hz" << std::endl;
        
        tf2_ros::StaticTransformBroadcaster static_broadcaster;
        ros::Rate rate(rate_hz);
        
        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            
            // Iterate through all transformations
            const auto& tf_map = tf_cache_.getTFMap();
            for (const auto &[child, vec] : tf_map)
            {
                for (const auto &[parent, tf] : vec)
                {
                    geometry_msgs::TransformStamped tf_msg;
                    tf_msg.header.stamp = now;
                    tf_msg.header.frame_id = parent;
                    tf_msg.child_frame_id = child;
                    
                    // Extract rotation and translation
                    Eigen::Matrix3d rot = tf.mat.block<3, 3>(0, 0);
                    Eigen::Quaterniond q(rot);
                    
                    tf_msg.transform.translation.x = tf.mat(0, 3);
                    tf_msg.transform.translation.y = tf.mat(1, 3);
                    tf_msg.transform.translation.z = tf.mat(2, 3);
                    
                    tf_msg.transform.rotation.x = q.x();
                    tf_msg.transform.rotation.y = q.y();
                    tf_msg.transform.rotation.z = q.z();
                    tf_msg.transform.rotation.w = q.w();
                    
                    static_broadcaster.sendTransform(tf_msg);
                }
            }
            
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ExtrinsicManager tf_cache_;
};


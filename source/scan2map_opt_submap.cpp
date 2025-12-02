/***********************************************************************
 * scan2map_opt.cpp
 * 
 * Scan-to-Map Optimization with Loop Closure Detection and Global Optimization
 * 
 * This implementation provides:
 * 1. Frame-by-frame scan-to-map optimization using point-to-plane constraints
 * 2. Submap construction around current pose for acceleration  
 * 3. Loop closure detection based on pose distance
 * 4. Global pose graph optimization when loop closures are detected
 * 5. Map and submap reconstruction after global optimization
 * 
 * Author: AI Assistant
 * Date: 2025-11-13
 ***********************************************************************/

// Core ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// Feature extraction
#include "feature_extractor.hpp"

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard includes
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>

// GTSAM includes
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

// Custom includes
#include "liorf/cloud_info.h"
#include "liorf/save_map.h"
#include "nano_gicp/nano_gicp.h"

using namespace gtsam;

// Define PointType instead of using utility.h
typedef pcl::PointXYZI PointType;

// Helper functions for angle conversion (from utility.h)
inline double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}



class Scan2MapOptimizer
{
public:
    // ROS publishers for visualization
    ros::NodeHandle nh;
    ros::Publisher pubKeyframeSubmap;
    ros::Publisher pubOptimizedCurrentFrame;
    
    // Point cloud metadata and poses (no actual point cloud data stored)
    std::vector<std::string> pointCloudFiles; // File paths for lazy loading
    std::vector<pcl::PointCloud<PointType>::Ptr> pointCloudCache; // cached loaded clouds
    std::mutex cacheMutex; // protects the cache vector
    std::vector<Pose3> initialPoses;
    std::vector<Pose3> optimizedPoses;
    std::vector<double> timestamps; // Timestamps for each frame
    std::string dataDirectory; // Base directory for data files
    
    // Global map and submap
    pcl::PointCloud<PointType>::Ptr globalMap;
    pcl::PointCloud<PointType>::Ptr currentSubmap;
    
    // Smart submap management for acceleration
    pcl::PointCloud<PointType>::Ptr smartSubmap;
    Point3 lastSubmapCenter;  // Last position where submap was built
    double submapUpdateDistance;  // Distance threshold to rebuild submap (10m)
    double submapSearchRadius;    // Radius for submap construction (250m)
    bool submapNeedsUpdate;       // Flag to indicate submap needs rebuilding
    
    // KD-tree for nearest neighbor search
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSubmap;
    
    // Voxel grid filters
    pcl::VoxelGrid<PointType> downSizeFilterScan;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    
    // GTSAM pose graph optimization
    NonlinearFactorGraph graph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    
    // Loop closure detection
    std::vector<std::pair<int, int>> loopClosures;
    double loopClosureThreshold;
    int minLoopClosureInterval;
    int lastLoopClosureFrame; // Track last frame where loop closure was detected
    
    // Optimization parameters
    int maxIterations;
    double convergenceThreshold;
    double submapRadius;
    double planeValidThreshold;
    double maxCorrespondenceDistance;
    
    // Current frame being processed
    int currentFrameId;
    int consecutiveRejectCount;  // 连续拒绝的帧数计数器
    
    // Translation offset (using first pose as reference)
    double tx_offset;
    double ty_offset;
    double tz_offset;
    
    // OpenMP parameters
    int numberOfCores;
    
    // Transformation matrices for current optimization
    float transformTobeMapped[6];
    bool isDegenerate;
    cv::Mat matP;
    
    // Point association vectors for parallel computation
    std::vector<PointType> laserCloudOriVec;
    std::vector<PointType> coeffSelVec;
    std::vector<bool> pointSelFlag;
    
    // Feature extraction
    // FeatureExtractor featureExtractor;
    
    Scan2MapOptimizer()
    {
        // Initialize GTSAM ISAM2
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);
        
        // Initialize point clouds
        globalMap.reset(new pcl::PointCloud<PointType>());
        currentSubmap.reset(new pcl::PointCloud<PointType>());
        smartSubmap.reset(new pcl::PointCloud<PointType>());
        kdtreeSubmap.reset(new pcl::KdTreeFLANN<PointType>());
        
        // Initialize smart submap parameters
        submapUpdateDistance = 2.0;   // 2 meters
        submapSearchRadius = 100.0;    // 100 meters
        submapNeedsUpdate = true;      // Force initial submap creation
        lastSubmapCenter = Point3(0, 0, 0);
        
        // Set voxel grid parameters
        downSizeFilterScan.setLeafSize(0.1, 0.1, 0.1);
        // downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);
        // downSizeFilterScan.setLeafSize(0.3, 0.3, 0.3);
        downSizeFilterMap.setLeafSize(0.4, 0.4, 0.4);   
        // Initialize parameters
        loopClosureThreshold = 8.0;  // 增加到8米以更好地检测漂移
        minLoopClosureInterval = 10;  // minimum 10 frames between loop closures
        lastLoopClosureFrame = -20;  // Initialize to allow immediate detection
        maxIterations = 50; // 30 ;
        convergenceThreshold = 1.0;  // Relaxed: 0.05 degrees and 0.05 c meters for better convergence
        submapRadius = 5.0;  // 10 meters
        planeValidThreshold = 0.1;   // Relaxed: 0.08m for more plane matches
        maxCorrespondenceDistance = 1.2; // Relaxed: 1.2m for more correspondences
        
        currentFrameId = 0; 
        consecutiveRejectCount = 0;
        tx_offset = 0.0;
        ty_offset = 0.0;
        tz_offset = 0.0;
        numberOfCores = 4;
        isDegenerate = false;
        
        // Initialize feature extraction with parameters
        // FeatureExtractor::FeatureExtractionParams featureParams;
        // featureParams.edgeThreshold = 0.1;
        // featureParams.surfThreshold = 0.1;
        // featureParams.odometrySurfLeafSize = 0.2;
        // featureParams.N_SCAN = 128;       // Typical for RS-128
        // featureParams.Horizon_SCAN = 1800; // Typical horizontal resolution
        // featureParams.useRingBasedExtraction = true; // Use geometry-based extraction instead of ring-based
        // // featureParams.useRingBasedExtraction = false; // Use geometry-based extraction instead of ring-based
        // featureExtractor.setParameters(featureParams);
        
        // Initialize transformation
        for (int i = 0; i < 6; ++i) {
            transformTobeMapped[i] = 0.0;
        }
        
        // Initialize ROS publishers
        pubKeyframeSubmap = nh.advertise<sensor_msgs::PointCloud2>("/liorf/keyframe_submap", 1);
        pubOptimizedCurrentFrame = nh.advertise<sensor_msgs::PointCloud2>("/liorf/optimized_current_frame", 1);
        
        ROS_INFO("Scan2MapOptimizer initialized");
    }
    
    ~Scan2MapOptimizer()
    {
        ROS_INFO("Cleaning up Scan2MapOptimizer...");
        
        // Clean up feature extractor
        // featureExtractor.cleanup();
        
        // Clean up point cloud cache
        {
            std::lock_guard<std::mutex> lock(cacheMutex);
            pointCloudCache.clear();
        }
        
        // Clean up GTSAM objects
        delete isam;
        
        ROS_INFO("Scan2MapOptimizer cleanup completed");
    }
    
    /**
     * Load metadata and poses from files (lazy loading for point clouds)
     */
    bool loadData(const std::string& dataDir)
    {
        dataDirectory = dataDir;
        
        ROS_INFO("Loading metadata from directory: %s", dataDir.c_str());
        
        std::string posesFile = dataDir + "/geo_key_pose_opt.tum";
        // std::string posesFile = dataDir + "/global_lidar_poses_opt.tum";
        std::ifstream file(posesFile);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open poses file: %s", posesFile.c_str());
            return false;
        }
        
        pointCloudFiles.clear();
        initialPoses.clear();
        optimizedPoses.clear();
        timestamps.clear();
        
        bool firstPose = true;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            
            if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
                continue;
            }

            // Store point cloud file path (don't load yet)
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3) << timestamp;
            std::string cloudFile = dataDir + "pcd/" + oss.str() + "_surf.pcd";
            
            // Check if file exists before adding to list
            std::ifstream checkFile(cloudFile);
            if (!checkFile.good()) {
                ROS_WARN("Point cloud file does not exist: %s", cloudFile.c_str());
                continue;
            }
            checkFile.close();
            
            // Set offset from first valid pose
            if (firstPose) {
                tx_offset = tx;
                ty_offset = ty;
                tz_offset = tz;
                firstPose = false;
                ROS_INFO("Setting translation offset: tx_offset=%.6f, ty_offset=%.6f", tx_offset, ty_offset);
            }
            
            // Apply offset to translation
            double tx_corrected = tx - tx_offset;
            double ty_corrected = ty - ty_offset;
            double tz_corrected = tz - tz_offset;

            // Create pose with offset applied
            Rot3 rotation = Rot3::Quaternion(qw, qx, qy, qz);
            Point3 translation(tx_corrected, ty_corrected, tz_corrected);
            Pose3 pose(rotation, translation);
            
            pointCloudFiles.push_back(cloudFile);
            initialPoses.push_back(pose);
            optimizedPoses.push_back(pose);
            timestamps.push_back(timestamp);
        }
        
        file.close();
        
        ROS_INFO("Loaded metadata for %d frames", (int)pointCloudFiles.size());

        // Initialize cache slots (empty) sized to number of frames
        {
            std::lock_guard<std::mutex> lock(cacheMutex);
            pointCloudCache.clear();
            pointCloudCache.resize(pointCloudFiles.size());
        }
        return !pointCloudFiles.empty();
    }

    /**
     * Thread-safe cached loader: return cached cloud if available, otherwise load and cache it.
     */
    pcl::PointCloud<PointType>::Ptr getCachedPointCloud(int frameId)
    {
        if (frameId < 0 || frameId >= (int)pointCloudFiles.size()) {
            ROS_ERROR("Invalid frame ID: %d (valid range: 0-%d)", frameId, (int)pointCloudFiles.size()-1);
            return nullptr;
        }

        // Fast path: check without locking for non-null pointer
        {
            std::lock_guard<std::mutex> lock(cacheMutex);
            if (pointCloudCache[frameId]) {
                return pointCloudCache[frameId];
            }
        }

        // Load outside of holding the slot (to avoid long lock duration)
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        if (pcl::io::loadPCDFile<PointType>(pointCloudFiles[frameId], *cloud) == -1) {
            ROS_ERROR("Failed to load point cloud: %s", pointCloudFiles[frameId].c_str());
            return nullptr;
        }

        // Extract corner and surface features from the loaded point cloud
        // Use the ring-based feature extraction method for PandarPointXYZIRT input
        // pcl::PointCloud<PointType>::Ptr cornerPoints(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr surfacePoints(new pcl::PointCloud<PointType>());
        // featureExtractor.extractFeaturesWithRing(cloud, cornerPoints, surfacePoints);
        // std::cout << "read " <<  cloud->size() << " original points " << std::endl;
        // std::cout << "read " <<  cornerPoints->size() << " cornerPoints points " << std::endl;
        // std::cout << "read " <<  surfacePoints->size() << " surfacePoints points " << std::endl;
        // pcl::io::savePCDFileBinary( "/home/tyjt/Desktop/ros_ws/cornerPoints.pcd", *cornerPoints);
        // pcl::io::savePCDFileBinary( "/home/tyjt/Desktop/ros_ws/surfacePoints.pcd", *surfacePoints); 

        // Convert surfacePoints (PointXYZI) to PointType cloud for further processing
        // pcl::PointCloud<PointType>::Ptr processedCloud(new pcl::PointCloud<PointType>());
        // processedCloud->clear();
        // *processedCloud = *surfacePoints; // Direct assignment since both are PointXYZI
        // std::cout << "read < surf > " <<  processedCloud->size() << " original points " << std::endl;

        // Optionally, you can use the extracted features here
        // For now, we'll continue with the processed cloud for scan-to-map optimization
        
        // std::cout << "read " <<  processedCloud->size() << " points " << std::endl;
        downSizeFilterScan.setInputCloud(cloud);
        downSizeFilterScan.filter(*cloud);
        std::cout << "Loaded and downsampled point cloud frame " << frameId 
                  << ": " << pointCloudFiles[frameId] 
                  << " (" << cloud->size() << " points after downsampling)" << std::endl;

        // Store into cache
        {
            std::lock_guard<std::mutex> lock(cacheMutex);
            pointCloudCache[frameId] = cloud;
        }

        ROS_DEBUG("Cached point cloud frame %d: %s (%d points)", frameId, pointCloudFiles[frameId].c_str(), (int)cloud->size());
        return cloud;
    }
    
    /**
     * Main optimization loop
     */
    void optimize()
    {
        if (pointCloudFiles.empty()) {
            ROS_ERROR("No data loaded for optimization");
            return;
        }
        
        ROS_INFO("Starting scan-to-map optimization for %d frames", (int)pointCloudFiles.size());
        
        // Add first pose as prior
        graph.add(PriorFactor<Pose3>(Symbol('x', 0), initialPoses[0], 
                  noiseModel::Diagonal::Sigmas((Vector(6) << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4).finished())));
        initialEstimate.insert(Symbol('x', 0), initialPoses[0]);
        
        // Load and add first frame to global map (use cache)
        pcl::PointCloud<PointType>::Ptr firstFrame = getCachedPointCloud(0);
        if (firstFrame) {
            pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*firstFrame, *transformedCloud, 
                                     initialPoses[0].matrix().cast<float>());
            *globalMap += *transformedCloud;
        } else {
            ROS_ERROR("Failed to load first frame for initialization");
            return;
        }
        
        // Process each frame
        for (currentFrameId = 1; currentFrameId < pointCloudFiles.size(); ++currentFrameId) {
        // for (currentFrameId = 1; currentFrameId < pointCloudFiles.size(); currentFrameId = currentFrameId+2) {
            ROS_INFO("Processing frame %d/%d for  %s", currentFrameId , (int)pointCloudFiles.size(), pointCloudFiles[currentFrameId].c_str());

            // Scan-to-map optimization for current frame (direct global map optimization)
            // if (currentFrameId > 3 && globalMap->size() > 1000) {
            // }
            scanToMapOptimization();

            // 每10帧进行一次滑动窗口优化来减少累积漂移
            if (currentFrameId % 10 == 0 && currentFrameId >= 10) {
                performSlidingWindowOptimization();
            }

            // Add optimized frame to global map
            addFrameToGlobalMap();
            
            // Check for loop closures (增加检测频率)
            detectLoopClosures();
            
            // Perform global optimization if loop closure detected
            if (!loopClosures.empty() && 
                loopClosures.back().first == currentFrameId) {
                performGlobalOptimization();
                reconstructGlobalMap();
            }
            
            // Add current pose to graph for next iteration
            if (currentFrameId < pointCloudFiles.size() - 1) {
                // Check if the key already exists before inserting
                Symbol currentSymbol('x', currentFrameId);
                if (!initialEstimate.exists(currentSymbol)) {
                    initialEstimate.insert(currentSymbol, optimizedPoses[currentFrameId]);
                } else {
                    initialEstimate.update(currentSymbol, optimizedPoses[currentFrameId]);
                }
            }
            
            // Progress update
            if (currentFrameId % 10 == 0) {
                ROS_INFO("Processed %d/%d frames", currentFrameId + 1, (int)pointCloudFiles.size());
            }
        }
        
        ROS_INFO("Scan-to-map optimization completed");
        
        // Final global optimization
        performGlobalOptimization();
        reconstructGlobalMap();
        
        // Save results
        saveResults();
    }
    
private:
    

    
    /**
     * Filter point cloud to keep only points within specified distance
     */
    void filterPointCloudByDistance(pcl::PointCloud<PointType>::Ptr& cloud, double maxDistance = 100.0)
    {
        if (cloud->empty()) {
            return;
        }
        
        pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>());
        int originalSize = cloud->size();
        for (const auto& point : cloud->points) {
            double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (point.z > 5.0) {
                continue;
            }
            if (distance <= maxDistance) {
                filteredCloud->points.push_back(point);
            }
        }
        
        filteredCloud->width = filteredCloud->points.size();
        filteredCloud->height = 1;
        filteredCloud->is_dense = true;
        
        cloud = filteredCloud;
        
        ROS_DEBUG("Filtered point cloud from %d to %d points (max distance: %.1f m)", 
                  originalSize, (int)cloud->size(), maxDistance);
    }
    
    /**
     * Publish point cloud to ROS topic
     */
    void publishPointCloud(ros::Publisher& publisher, pcl::PointCloud<PointType>::Ptr cloud, const std::string& frame_id = "map")
    {
        if (cloud->empty()) {
            return;
        }
        
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(*cloud, cloudMsg);
        cloudMsg.header.stamp = ros::Time::now();
        cloudMsg.header.frame_id = frame_id;
        publisher.publish(cloudMsg);
    }
    
    /**
     * Smart submap construction around current pose for acceleration
     * Only rebuilds when position has moved more than submapUpdateDistance
     */
    void buildSmartSubmap()
    {
        Point3 currentPos = optimizedPoses[currentFrameId].translation();
        
        // Check if we need to update the submap
        if (!submapNeedsUpdate) {
            double distanceFromLastCenter = (currentPos - lastSubmapCenter).norm();
            if (distanceFromLastCenter < submapUpdateDistance) {
                // No need to rebuild submap, position hasn't moved enough
                return;
            }
        }
        
        // 如果连续拒绝较多，使用更保守的子地图策略
        double effectiveRadius = submapSearchRadius;
        if (consecutiveRejectCount > 5) {
            effectiveRadius = submapSearchRadius * 1.5;  // Increase radius by 50%
            ROS_WARN("Using increased submap radius %.1f m due to %d consecutive rejections", 
                     effectiveRadius, consecutiveRejectCount);
        }
        
        ROS_INFO("Building smart submap around position [%.1f, %.1f, %.1f] with radius %.1f m", 
                 currentPos.x(), currentPos.y(), currentPos.z(), effectiveRadius);
        
        smartSubmap->clear();
        
        // Use spatial filtering to extract points within radius
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        for (int i = 0; i < globalMap->points.size(); ++i) {
            const PointType& point = globalMap->points[i];

            if(std::fabs(point.x - currentPos.x()) < effectiveRadius * 1.25
               && std::fabs(point.y - currentPos.y()) < effectiveRadius * 1.25)
               {
                inliers->indices.push_back(i);
               }
 
        }
        
        // Extract points within radius
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(globalMap);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*smartSubmap);
        ROS_INFO("Smart submap built with %d points  ", (int)smartSubmap->size() );
        // Downsample the smart submap for efficiency
        // if (smartSubmap->size() > 10000) {
        if ( 1 ) {
            pcl::PointCloud<PointType>::Ptr downsampledSubmap(new pcl::PointCloud<PointType>());
            downSizeFilterMap.setInputCloud(smartSubmap);
            downSizeFilterMap.filter(*downsampledSubmap);
            smartSubmap = downsampledSubmap;
        }
        
        // Update submap management variables
        lastSubmapCenter = currentPos;
        submapNeedsUpdate = false;
        
        ROS_INFO("Smart submap built with %d points (extracted from %d global points)", 
                 (int)smartSubmap->size(), (int)globalMap->size());
    }
    
    /**
     * Smart submap construction based on nearest 10 keyframes
     * Finds the 10 keyframes closest to current position and builds submap from them
     */
    void buildSmartSubmap_withlast10keyframe()
    {
        Point3 currentPos = optimizedPoses[currentFrameId].translation();
        
        ROS_INFO("Building smart submap based on nearest 10 keyframes around position [%.1f, %.1f, %.1f]", 
                 currentPos.x(), currentPos.y(), currentPos.z());
        
        smartSubmap->clear();
        
        // Calculate distances to all previous frames
        std::vector<std::pair<double, int>> frameDistances;
        for (int i = 0; i < currentFrameId; ++i) {
            Point3 framePos = optimizedPoses[i].translation();
            double distance = (currentPos - framePos).norm();
            frameDistances.push_back(std::make_pair(distance, i));
        }
        
        // Sort by distance to find nearest frames
        std::sort(frameDistances.begin(), frameDistances.end());
        
        // Select up to 10 nearest keyframes
        int numKeyframes = std::min(10, (int)frameDistances.size());
        std::vector<int> selectedKeyframes;
        
        for (int i = 0; i < numKeyframes; ++i) {
            selectedKeyframes.push_back(frameDistances[i].second);
        }
        
        ROS_INFO("Selected %d nearest keyframes for submap construction", numKeyframes);
        
        // Build submap from selected keyframes
        int totalPointsAdded = 0;
        for (int keyframeId : selectedKeyframes) {
            // Load keyframe on demand (cached)
            pcl::PointCloud<PointType>::Ptr keyframeCloud = getCachedPointCloud(keyframeId);
            if (!keyframeCloud || keyframeCloud->empty()) {
                ROS_WARN("Failed to load keyframe %d for submap construction", keyframeId);
                continue;
            }
            
            // Transform keyframe to its optimized pose
            pcl::PointCloud<PointType>::Ptr transformedKeyframe(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*keyframeCloud, *transformedKeyframe, 
                                   optimizedPoses[keyframeId].matrix().cast<float>());
            
            // Add transformed keyframe to submap
            *smartSubmap += *transformedKeyframe;
            totalPointsAdded += transformedKeyframe->size();
            
            // ROS_DEBUG("Added keyframe %d to submap (distance: %.2f m, points: %d)", 
            //          keyframeId, frameDistances[std::find_if(frameDistances.begin(), frameDistances.end(),
            //          [keyframeId](const std::pair<double, int>& p) { return p.second == keyframeId; })]->first,
            //          (int)transformedKeyframe->size());
        }
        
        ROS_INFO("Keyframe-based submap built with %d points from %d keyframes", 
                 totalPointsAdded, numKeyframes);
        
        // Downsample the keyframe-based submap for efficiency
        if (smartSubmap->size() > 0) {
            pcl::PointCloud<PointType>::Ptr downsampledSubmap(new pcl::PointCloud<PointType>());
            downSizeFilterMap.setInputCloud(smartSubmap);
            downSizeFilterMap.filter(*downsampledSubmap);
            smartSubmap = downsampledSubmap;
            
            ROS_INFO("Final keyframe-based submap after downsampling: %d points", 
                     (int)smartSubmap->size());
        }
        
        // Update submap management variables
        lastSubmapCenter = currentPos;
        submapNeedsUpdate = false;
    }
    
    /**
     * Scan-to-map optimization using point-to-plane constraints
     * Based on the scan2MapOptimization() function from mapOptmization.cpp
     */
    void scanToMapOptimization()
    {
        // Check if global map has enough points for optimization
        if (globalMap->size() < 1000) {
            ROS_WARN("Global map too small (%d points) for frame %d", (int)globalMap->size(), currentFrameId);
            return;
        }
        
        // Build smart submap for efficient nearest neighbor search
        // buildSmartSubmap_withlast10keyframe();

        buildSmartSubmap();

        // Check if smart submap has enough points
        if (smartSubmap->size() < 5000) {
            ROS_WARN("Smart submap too small (%d points) for frame %d, using global map", 
                     (int)smartSubmap->size(), currentFrameId);
            kdtreeSubmap->setInputCloud(globalMap);
        } else {
            // Build KD-tree for smart submap
            kdtreeSubmap->setInputCloud(smartSubmap);
            ROS_INFO("Using smart submap with %d points for frame %d optimization", 
                     (int)smartSubmap->size(), currentFrameId);
        }
        
        // Smart initialization with fallback strategy
        Pose3 initialPose = initialPoses[currentFrameId];     // 初始位姿，用于异常检测和fallback
        Pose3 currentPose = optimizedPoses[currentFrameId];   // 当前优化位姿
 
        // 使用多种位姿预测策略
        Pose3 motionModelPose = optimizedPoses[currentFrameId-1] * ( initialPoses[currentFrameId-1].inverse() * initialPose );
        
        // 使用速度模型作为额外约束
        Pose3 velocityModelPose = initialPose;
        if (currentFrameId >= 2) {
            Pose3 relativePose1 = optimizedPoses[currentFrameId-2].inverse() * optimizedPoses[currentFrameId-1];
            velocityModelPose = optimizedPoses[currentFrameId-1] * relativePose1;
        }
        
        // 选择更稳定的预测作为起始点
        Point3 motionTrans = motionModelPose.translation();
        Point3 velocityTrans = velocityModelPose.translation();
        double predictionDiff = (motionTrans - velocityTrans).norm();
        
        const auto optimizationStartPose = (predictionDiff < 2.0) ? motionModelPose : initialPose;
        ROS_DEBUG("Frame %d: Motion model diff=%.3fm, using %s prediction", 
                  currentFrameId, predictionDiff, (predictionDiff < 2.0) ? "motion" : "initial");
        
        auto rpy = optimizationStartPose.rotation().rpy();  
        transformTobeMapped[0] = rpy(0); // roll
        transformTobeMapped[1] = rpy(1); // pitch
        transformTobeMapped[2] = rpy(2); // yaw 
        transformTobeMapped[3] = optimizationStartPose.translation().x(); 
        transformTobeMapped[4] = optimizationStartPose.translation().y(); 
        transformTobeMapped[5] = optimizationStartPose.translation().z();

        // Load current frame on demand (cached)
        pcl::PointCloud<PointType>::Ptr currentFrame = getCachedPointCloud(currentFrameId);
        if (!currentFrame) {
            ROS_ERROR("Failed to load current frame %d for optimization", currentFrameId);
            return;
        }
        
        // Filter current frame to keep only points within 100m
        int originalSize = currentFrame->size();
        
        filterPointCloudByDistance(currentFrame, 100.0);
        downSizeFilterScan.setInputCloud(currentFrame);
        downSizeFilterScan.filter(*currentFrame);


        ROS_INFO("Frame %d: filtered from %d to %d points (kept points within 100m)", 
                 currentFrameId, originalSize, (int)currentFrame->size());
        
        // Prepare vectors for parallel computation
        int scanSize = currentFrame->size();
        laserCloudOriVec.resize(scanSize);
        coeffSelVec.resize(scanSize);
        pointSelFlag.assign(scanSize, false);
        
        // Iterative optimization with divergence detection
        ROS_INFO("Starting scan-to-map optimization for frame %d", currentFrameId);
        
        for (int iterCount = 0; iterCount < maxIterations; iterCount++) {

            // Find point-to-plane correspondences
            // ok 使用原始版本不动任何参数
            surfOptimization(currentFrame);
            
            // Combine optimization coefficients
            combineOptimizationCoeffs(laserCloudOriVec, coeffSelVec);
            
            // Perform Levenberg-Marquardt optimization
            if (LMOptimization(iterCount)) {
                // Converged, exit optimization loop
                break;
            }
            
        }
        
        // Create candidate optimized pose
        Rot3 optimizedRotation = Rot3::RzRyRx(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]); // roll, pitch, yaw
        Point3 optimizedTranslation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        Pose3 candidateOptimizedPose(optimizedRotation, optimizedTranslation);
        
        // Sanity check on the optimized pose change (与初始位姿比较)
        Point3 beforeTranslation = initialPose.translation();  // 使用初始位姿作为基准
        Point3 afterTranslation = candidateOptimizedPose.translation();
        auto beforeRPY = initialPose.rotation().rpy();  // 使用初始位姿作为基准
        auto afterRPY = candidateOptimizedPose.rotation().rpy();
        
        float translationChange = sqrt(pow(afterTranslation.x() - beforeTranslation.x(), 2) +
                                     pow(afterTranslation.y() - beforeTranslation.y(), 2) +
                                     pow(afterTranslation.z() - beforeTranslation.z(), 2));
        
        float rotationChange = sqrt(pow(afterRPY(0) - beforeRPY(0), 2) +
                                  pow(afterRPY(1) - beforeRPY(1), 2) +
                                  pow(afterRPY(2) - beforeRPY(2), 2)) * 180.0 / M_PI;
        
        // 动态调整阈值以减少过度拒绝同时控制漂移
        float maxTranslationChange = 0.5; // 基础阈值：50cm
        float maxRotationChange = 1.5;    // 基础阈值：1.5度
        
        // 根据连续拒绝次数动态调整阈值
        if (consecutiveRejectCount > 3) {
            maxTranslationChange = std::min(1.0f, maxTranslationChange * (1.0f + consecutiveRejectCount * 0.2f));
            maxRotationChange = std::min(3.0f, maxRotationChange * (1.0f + consecutiveRejectCount * 0.2f));
            ROS_DEBUG("Relaxed thresholds: trans=%.2f, rot=%.2f (consecutive rejects: %d)", 
                      maxTranslationChange, maxRotationChange, consecutiveRejectCount);
        }
        
        // 额外的一致性检查：与运动模型预测的差异
        Point3 predictedPos = optimizationStartPose.translation();
        float predictionConsistency = (afterTranslation - predictedPos).norm();
        bool consistentWithPrediction = predictionConsistency < 1.0; // 1米一致性阈值

        bool acceptOptimizedPose = true;
        std::string rejectReason = "";
        
        if (translationChange > maxTranslationChange) {
            acceptOptimizedPose = false;
            rejectReason += "excessive translation (" + std::to_string(translationChange) + "m > " + std::to_string(maxTranslationChange) + "m); ";
        }

        if (rotationChange > maxRotationChange) {
            acceptOptimizedPose = false;
            rejectReason += "excessive rotation (" + std::to_string(rotationChange) + "deg > " + std::to_string(maxRotationChange) + "deg); ";
        }
        
        if (!consistentWithPrediction) {
            acceptOptimizedPose = false;
            rejectReason += "inconsistent with motion model (" + std::to_string(predictionConsistency) + "m); ";
        }
 
        // Apply optimized pose only if it passes sanity checks
        static int init_pose_cnt = 0;
        if (acceptOptimizedPose) {
            optimizedPoses[currentFrameId] = candidateOptimizedPose;
            consecutiveRejectCount = 0;  // 重置连续拒绝计数器
            ROS_INFO("Frame %d: Optimization accepted and applied", currentFrameId);
        } else {
            // 使用初始位姿，确保该帧位置不更新
            // optimizedPoses[currentFrameId] = initialPose;
            optimizedPoses[currentFrameId] = optimizationStartPose;
            
            consecutiveRejectCount++;
            ROS_ERROR("===================  %d   ==================", init_pose_cnt);
            ROS_ERROR("Frame %d: Using initial pose due to excessive optimization change (consecutive: %d)", 
                     currentFrameId, consecutiveRejectCount);
            init_pose_cnt++;
            
            // 如果连续拒绝超过10帧，强制重建子地图
            // if (consecutiveRejectCount >= 10) {
            //     ROS_ERROR("Frame %d: Too many consecutive rejections (%d), forcing submap rebuild", 
            //               currentFrameId, consecutiveRejectCount);
            //     submapNeedsUpdate = true;  // 强制重建子地图
            //     consecutiveRejectCount = 0;
            // }
        }
        
        // 输出详细的优化结果
        Pose3 finalPose = optimizedPoses[currentFrameId]; // This might be original or optimized
        Point3 finalTranslation = finalPose.translation();
        auto finalRPY = finalPose.rotation().rpy();
        
        ROS_WARN("=== Frame %d Optimization Results ===", currentFrameId);
        ROS_WARN("Initial: T=[%.3f, %.3f, %.3f], RPY=[%.3f, %.3f, %.3f]", 
                 beforeTranslation.x(), beforeTranslation.y(), beforeTranslation.z(),
                 beforeRPY(0)*180/M_PI, beforeRPY(1)*180/M_PI, beforeRPY(2)*180/M_PI);
        ROS_WARN("Final:   T=[%.3f, %.3f, %.3f], RPY=[%.3f, %.3f, %.3f] %s", 
                 finalTranslation.x(), finalTranslation.y(), finalTranslation.z(),
                 finalRPY(0)*180/M_PI, finalRPY(1)*180/M_PI, finalRPY(2)*180/M_PI,
                 acceptOptimizedPose ? "[OPTIMIZED]" : "[REJECTED - USING INITIAL]");
        ROS_WARN("Change: T=[%.3f, %.3f, %.3f] (%.3fm total)", 
                 finalTranslation.x()-beforeTranslation.x(), 
                 finalTranslation.y()-beforeTranslation.y(), 
                 finalTranslation.z()-beforeTranslation.z(),
                 translationChange);
        ROS_WARN("Angles: R=[%.3f], P=[%.3f], Y=[%.3f] deg (Total: %.3fdeg)", 
                 (finalRPY(0)-beforeRPY(0))*180/M_PI, 
                 (finalRPY(1)-beforeRPY(1))*180/M_PI, 
                 (finalRPY(2)-beforeRPY(2))*180/M_PI,
                 rotationChange);
        
        // Publish keyframe submap for visualization
        publishPointCloud(pubKeyframeSubmap, smartSubmap, "map");
        
        // Create and publish optimized current frame using the final pose
        pcl::PointCloud<PointType>::Ptr optimizedCurrentFrame(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*currentFrame, *optimizedCurrentFrame, finalPose.matrix().cast<float>());
        publishPointCloud(pubOptimizedCurrentFrame, optimizedCurrentFrame, "map");
        
        ROS_INFO("Published keyframe submap (%d points) and optimized current frame (%d points)", 
                 (int)smartSubmap->size(), (int)optimizedCurrentFrame->size());
    }
 
    /**
     * Surface feature optimization (point-to-plane)
     * Restored from original mapOptmization.cpp implementation
     */
    void surfOptimization(pcl::PointCloud<PointType>::Ptr& currentScan)
    {
        updatePointAssociateToMap();

        int use_pts_cnt = 0;
        #pragma omp parallel for num_threads(numberOfCores)
        // for (int i = 0; i < currentScan->size(); i++)
        for (int i = 0; i < currentScan->size(); i = i+3)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = currentScan->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSubmap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            
            if ( validateSurfacePatch( pointSearchInd, smartSubmap ) )
            {
                use_pts_cnt++;
            }
            else
            {
                continue;
            }

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    // Use smartSubmap if available, otherwise use globalMap
                    if (smartSubmap->size() >= 500) {
                        matA0(j, 0) = smartSubmap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = smartSubmap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = smartSubmap->points[pointSearchInd[j]].z;
                    } else {
                        matA0(j, 0) = globalMap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = globalMap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = globalMap->points[pointSearchInd[j]].z;
                    }
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    float planeDist;
                    if (smartSubmap->size() >= 500) {
                        planeDist = fabs(pa * smartSubmap->points[pointSearchInd[j]].x +
                                       pb * smartSubmap->points[pointSearchInd[j]].y +
                                       pc * smartSubmap->points[pointSearchInd[j]].z + pd);
                    } else {
                        planeDist = fabs(pa * globalMap->points[pointSearchInd[j]].x +
                                       pb * globalMap->points[pointSearchInd[j]].y +
                                       pc * globalMap->points[pointSearchInd[j]].z + pd);
                    }
                    
                    if (planeDist > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriVec[i] = pointOri;
                        coeffSelVec[i] = coeff;
                        pointSelFlag[i] = true;
                    }
                }
            }
        }
        ROS_WARN("Frame %d: Surface feature optimization used %d/%d points", 
                 currentFrameId, use_pts_cnt, (int)currentScan->size());
    }
    
    /**
     * Update point association transformation matrix
     */
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                         transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    }
    
    /**
     * Transform point from lidar frame to map frame
     */
    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        Eigen::Vector3f point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3f point_w = transPointAssociateToMap * point_curr;
        
        po->x = point_w.x();
        po->y = point_w.y(); 
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }
    
    /**
     * Combine optimization coefficients from parallel computation
     */
    void combineOptimizationCoeffs(std::vector<PointType>& laserCloudOriVec, 
                                  std::vector<PointType>& coeffSelVec)
    {
        laserCloudOriVec.clear();
        coeffSelVec.clear();
        
        for (int i = 0; i < pointSelFlag.size(); ++i) {
            if (pointSelFlag[i]) {
                laserCloudOriVec.push_back(laserCloudOriVec[i]);
                coeffSelVec.push_back(coeffSelVec[i]);
            }
        }
        std::fill(pointSelFlag.begin(), pointSelFlag.end(), false);
    }
    
    /**
     * Levenberg-Marquardt optimization
     * Adapted from LMOptimization() in mapOptmization.cpp
     */

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[2]);
        float crx = cos(transformTobeMapped[2]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOriVec.size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOriVec[i].x;
            pointOri.y = laserCloudOriVec[i].y;
            pointOri.z = laserCloudOriVec[i].z;
            // lidar -> camera
            coeff.x = coeffSelVec[i].x;
            coeff.y = coeffSelVec[i].y;
            coeff.z = coeffSelVec[i].z;
            coeff.intensity = coeffSelVec[i].intensity;
            // in camera
/*             float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
             */

            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                      + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

            float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                      + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                      + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

            float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                      + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                      + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
              
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        ROS_WARN("Frame %d Iteration %d: deltaR=%.6f deg, deltaT=%.6f cm", 
                 currentFrameId, iterCount, deltaR, deltaT);

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    /**
     * Add current optimized frame to global map (with lazy loading)
     */
    void addFrameToGlobalMap()
    {
        // Load current frame on demand (cached)
        pcl::PointCloud<PointType>::Ptr currentFrame = getCachedPointCloud(currentFrameId);
        if (!currentFrame) {
            ROS_WARN("Failed to load frame %d for adding to global map", currentFrameId);
            return;
        }
        
        // Filter current frame to keep only points within 100m
        // filterPointCloudByDistance(currentFrame, 100.0);
        
        pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*currentFrame, *transformedCloud,
                                optimizedPoses[currentFrameId].matrix().cast<float>());
        *globalMap += *transformedCloud;
        
        // Mark that submap may need updating since global map has changed
        submapNeedsUpdate = true;
        
        ROS_INFO("Added frame %d to global map, total points: %d", 
                 currentFrameId, (int)globalMap->size());
    }
    
    /**
     * Detect loop closures based on pose distance and yaw angle
     */
    /**
     * Extract keyframes around a given frame index for GICP alignment
     */
    pcl::PointCloud<PointType>::Ptr extractKeyFrameSubmap(int keyFrameIndex, int numNeighbors)
    {
        pcl::PointCloud<PointType>::Ptr keyframeSubmap(new pcl::PointCloud<PointType>());
        
        // Extract point clouds from neighboring frames
        for (int i = -numNeighbors; i <= numNeighbors; ++i) {
            int frameIdx = keyFrameIndex + i;
            if (frameIdx < 0 || frameIdx >= pointCloudFiles.size()) {
                continue;
            }
            
            // Load frame on demand (cached)
            pcl::PointCloud<PointType>::Ptr frameCloud = getCachedPointCloud(frameIdx);
            if (!frameCloud || frameCloud->empty()) {
                continue;
            }
            
            // Filter frame to keep only points within 100m
            // filterPointCloudByDistance(frameCloud, 100.0);
            
            // Transform frame to its pose
            pcl::PointCloud<PointType>::Ptr transformedFrame(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*frameCloud, *transformedFrame, 
                                   optimizedPoses[frameIdx].matrix().cast<float>());
            
            *keyframeSubmap += *transformedFrame;
        }
        
        // Downsample the combined keyframe submap
        if (!keyframeSubmap->empty()) {
            pcl::PointCloud<PointType>::Ptr downsampledSubmap(new pcl::PointCloud<PointType>());
            downSizeFilterMap.setInputCloud(keyframeSubmap);
            downSizeFilterMap.filter(*downsampledSubmap);
            keyframeSubmap = downsampledSubmap;
        }
        
        return keyframeSubmap;
    }
    
    /**
     * Perform GICP-based loop closure refinement
     */
    bool performGICPLoopClosure(int currentFrameId, int candidateFrameId, Pose3& refinedRelativePose, double& fitnessScore)
    {
        ROS_INFO("Performing GICP loop closure between frames %d and %d", currentFrameId, candidateFrameId);
        
        // Extract keyframe submaps around both frames
        pcl::PointCloud<PointType>::Ptr currentSubmap = extractKeyFrameSubmap(currentFrameId, 0); // Only current frame
        pcl::PointCloud<PointType>::Ptr candidateSubmap = extractKeyFrameSubmap(candidateFrameId, 5); // Candidate + neighbors
        
        if (currentSubmap->size() < 300 || candidateSubmap->size() < 1000) {
            ROS_WARN("Insufficient points for GICP: current=%d, candidate=%d", 
                     (int)currentSubmap->size(), (int)candidateSubmap->size());
            return false;
        }
        
        // Calculate initial guess from current pose estimates
        Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
        {
            Pose3 currentPose = optimizedPoses[currentFrameId];
            Pose3 candidatePose = optimizedPoses[candidateFrameId];
            Pose3 relativePoseGuess = candidatePose.inverse() * currentPose;
            initialGuess = relativePoseGuess.matrix();
        }
        
        // Validate and sanitize initial guess
        bool guessValid = true;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!std::isfinite(initialGuess(r, c))) {
                    guessValid = false;
                    break;
                }
            }
            if (!guessValid) break;
        }
        
        if (!guessValid) {
            ROS_WARN("GICP: invalid initial guess detected, using identity");
            initialGuess.setIdentity();
        } else {
            // Orthonormalize rotation block to avoid numerical issues
            Eigen::Matrix3d R = initialGuess.block<3,3>(0,0);
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d U = svd.matrixU();
            Eigen::Matrix3d V = svd.matrixV();
            Eigen::Matrix3d R_ortho = U * V.transpose();
            if (R_ortho.determinant() < 0) {
                R_ortho = U * (Eigen::Vector3d(1,1,-1).asDiagonal()) * V.transpose();
            }
            initialGuess.block<3,3>(0,0) = R_ortho;
        }
        
        // Configure GICP
        nano_gicp::NanoGICP<PointType, PointType> gicp;
        gicp.setCorrespondenceRandomness(128);
        gicp.setMaxCorrespondenceDistance(1.0);
        gicp.setMaximumIterations(128);
        gicp.setTransformationEpsilon(1e-3);
        gicp.setRotationEpsilon(1e-3);
        gicp.setInitialLambdaFactor(1e-9);
        gicp.setRegularizationMethod(nano_gicp::RegularizationMethod::PLANE);
        
        bool success = false;
        try {
            // Set input clouds
            gicp.setInputSource(currentSubmap);
            gicp.setInputTarget(candidateSubmap);
            gicp.calculateSourceCovariances();
            gicp.calculateTargetCovariances();
            
            // Perform alignment
            pcl::PointCloud<PointType> aligned;
            gicp.align(aligned, initialGuess.cast<float>());
            
            // Check convergence and fitness
            success = gicp.hasConverged();
            fitnessScore = gicp.getFitnessScore();
            
            if (success && fitnessScore < 0.1) { // Fitness threshold
                // Extract refined transformation
                Eigen::Matrix4f refinedTransform = gicp.getFinalTransformation();
                
                // Convert to gtsam Pose3
                Eigen::Matrix4d refinedTransformD = refinedTransform.cast<double>();
                std::cout << "Refined GICP Transform:\n" << refinedTransformD << std::endl;
                refinedRelativePose = Pose3(refinedTransformD);
                
                ROS_INFO("GICP loop closure successful: fitness=%.3f", fitnessScore);
                return true;
            } else {
                ROS_WARN("GICP loop closure failed: converged=%d, fitness=%.3f", success, fitnessScore);
                return false;
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("GICP alignment exception: %s", e.what());
            return false;
        }
    }
    
    /**
     * 位姿平滑处理 - 减少优化后位姿的突变
     */
    Pose3 applyCandidatePoseSmoothing(const Pose3& candidatePose, int frameId)
    {
        if (frameId < 2) return candidatePose;
        
        // 使用加权平均进行平滑
        const double alpha = 0.7; // 平滑系数，越小越平滑
        
        // 获取前一帧的位姿和运动模型预测位姿
        Pose3 prevPose = optimizedPoses[frameId - 1];
        Pose3 initialPose = initialPoses[frameId];
        Pose3 predictedPose = optimizedPoses[frameId-1] * (initialPoses[frameId-1].inverse() * initialPose);
        
        // 对平移进行平滑
        Point3 candidateTranslation = candidatePose.translation();
        Point3 predictedTranslation = predictedPose.translation();
        Point3 smoothedTranslation = candidateTranslation * alpha + predictedTranslation * (1.0 - alpha);
        
        // 对旋转进行平滑（四元数球面插值）
        Quaternion candidateQuat = candidatePose.rotation().toQuaternion();
        Quaternion predictedQuat = predictedPose.rotation().toQuaternion();
        Quaternion smoothedQuat = candidateQuat.slerp(1.0 - alpha, predictedQuat);
        
        return Pose3(Rot3(smoothedQuat), smoothedTranslation);
    }

    /**
     * 滑动窗口优化 - 定期优化最近的N个位姿来减少累积漂移
     */
    void performSlidingWindowOptimization()
    {
        const int windowSize = 15; // 滑动窗口大小
        if (currentFrameId < windowSize) return;
        
        ROS_INFO("Performing sliding window optimization for frames %d-%d", 
                 currentFrameId - windowSize + 1, currentFrameId);
        
        // 创建滑动窗口的因子图
        NonlinearFactorGraph slidingGraph;
        Values slidingInitial;
        
        // 添加窗口内的位姿
        for (int i = currentFrameId - windowSize + 1; i <= currentFrameId; ++i) {
            slidingInitial.insert(Symbol('x', i), optimizedPoses[i]);
        }
        
        // 添加固定的先验因子（窗口起始位姿）
        int startIdx = currentFrameId - windowSize + 1;
        slidingGraph.add(PriorFactor<Pose3>(Symbol('x', startIdx), optimizedPoses[startIdx], 
                        noiseModel::Diagonal::Sigmas((Vector(6) << 1e3, 1e3, 1e3, 1e-4, 1e-4, 1e-4).finished())));
        
        // 添加相邻帧之间的里程计约束
        for (int i = startIdx; i < currentFrameId; ++i) {
            Pose3 relativePose = optimizedPoses[i].inverse() * optimizedPoses[i+1];
            slidingGraph.add(BetweenFactor<Pose3>(Symbol('x', i), Symbol('x', i+1), relativePose,
                           noiseModel::Diagonal::Sigmas((Vector(6) << 0.001, 0.001, 0.001, 0.05, 0.05, 0.05).finished())));
        }
        
        // 执行优化
        try {
            LevenbergMarquardtOptimizer slidingOptimizer(slidingGraph, slidingInitial);
            Values slidingResult = slidingOptimizer.optimize();
            
            // 更新优化后的位姿
            for (int i = startIdx; i <= currentFrameId; ++i) {
                optimizedPoses[i] = slidingResult.at<Pose3>(Symbol('x', i));
            }
            
            ROS_INFO("Sliding window optimization completed successfully");
        } catch (const std::exception& e) {
            ROS_WARN("Sliding window optimization failed: %s", e.what());
        }
    }

    void detectLoopClosures()
    {
        if (currentFrameId < minLoopClosureInterval) {
            return;
        }
        
        // 更频繁的回环检测以及时纠正漂移
        if (currentFrameId - lastLoopClosureFrame < 3) {
            return;
        }
        
        Point3 currentPos = optimizedPoses[currentFrameId].translation();
        Rot3 currentRot = optimizedPoses[currentFrameId].rotation();
        
        // Extract yaw angle from current pose
        auto currentRPY = currentRot.rpy();
        double currentYaw = currentRPY(2); // yaw is the third component
        
        for (int i = 0; i < currentFrameId - minLoopClosureInterval; ++i) {
            Point3 candidatePos = optimizedPoses[i].translation();
            double distance = (currentPos - candidatePos).norm();
            
            if (distance < loopClosureThreshold) {
                // Check yaw angle difference
                Rot3 candidateRot = optimizedPoses[i].rotation();
                auto candidateRPY = candidateRot.rpy();
                double candidateYaw = candidateRPY(2);
                
                // Calculate yaw difference (handle angle wrapping)
                double yawDiff = fabs(currentYaw - candidateYaw);
                if (yawDiff > M_PI) {
                    yawDiff = 2 * M_PI - yawDiff;
                }
                double yawDiffDegrees = yawDiff * 180.0 / M_PI;
                
                // Only proceed if yaw difference is less than 10 degrees
                if (yawDiffDegrees < 10.0) {
                    // Check if this loop closure already exists
                    bool alreadyExists = false;
                    for (const auto& lc : loopClosures) {
                        if ((lc.first == currentFrameId && lc.second == i) ||
                            (lc.first == i && lc.second == currentFrameId)) {
                            alreadyExists = true;
                            break;
                        }
                    }
                    
                    if (!alreadyExists) {
                        // Perform GICP-based loop closure refinement
                        Pose3 refinedRelativePose;
                        double fitnessScore;
                        
                        if (performGICPLoopClosure(currentFrameId, i, refinedRelativePose, fitnessScore)) {
                            // GICP refinement successful, add refined constraint
                            loopClosures.push_back(std::make_pair(currentFrameId, i));
                            
                            // Create noise model based on GICP fitness score
                            double noiseFactor = std::max(0.05, fitnessScore); // Minimum noise of 0.05
                            Vector6 noiseVector;
                            noiseVector << noiseFactor*noiseFactor, noiseFactor*noiseFactor, noiseFactor*noiseFactor, 
                                          noiseFactor, noiseFactor, noiseFactor;
                            auto noise = noiseModel::Diagonal::Variances(noiseVector);
                            
                            // Add refined constraint to graph
                            graph.add(BetweenFactor<Pose3>(Symbol('x', i), Symbol('x', currentFrameId), refinedRelativePose, noise));
                            
                            // Update last loop closure frame to skip next 10 frames
                            lastLoopClosureFrame = currentFrameId;
                            
                            ROS_INFO("GICP loop closure added: frame %d <-> frame %d (distance: %.2f m, yaw diff: %.1f°, fitness: %.3f)", 
                                     currentFrameId, i, distance, yawDiffDegrees, fitnessScore);
                            break; // Only add one loop closure per frame
                        } else {
                            ROS_WARN("GICP loop closure rejected between frames %d and %d", currentFrameId, i);
                        }
                    }
                }
            }
        }
    }
    
    /**
     * Perform global pose graph optimization
     */
    void performGlobalOptimization()
    {
        ROS_INFO("Performing global optimization with %d poses and %d loop closures", 
                 (int)optimizedPoses.size(), (int)loopClosures.size());
        
        // Create a new graph and values for global optimization
        NonlinearFactorGraph globalGraph;
        Values globalValues;
        
        // Add prior factor for first pose
        globalGraph.add(PriorFactor<Pose3>(Symbol('x', 0), optimizedPoses[0],
                  noiseModel::Diagonal::Sigmas((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished())));
        
        // Add all poses to initial estimate (only up to current frame)
        int maxFrameId = std::min(currentFrameId + 1, (int)optimizedPoses.size());
        for (int i = 0; i < maxFrameId; ++i) {
            globalValues.insert(Symbol('x', i), optimizedPoses[i]);
        }
        
        // Add odometry constraints
        auto odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
        for (int i = 1; i < maxFrameId; ++i) {
            Pose3 relativePose = optimizedPoses[i-1].inverse() * optimizedPoses[i];
            globalGraph.add(BetweenFactor<Pose3>(Symbol('x', i-1), Symbol('x', i), relativePose, odometryNoise));
        }
        
        // Add loop closure constraints
        auto loopNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
        for (const auto& lc : loopClosures) {
            if (lc.first < maxFrameId && lc.second < maxFrameId) {
                Pose3 relativePose = optimizedPoses[lc.second].inverse() * optimizedPoses[lc.first];
                globalGraph.add(BetweenFactor<Pose3>(Symbol('x', lc.second), Symbol('x', lc.first), relativePose, loopNoise));
            }
        }
        
        // Optimize using Levenberg-Marquardt
        LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");
        params.setMaxIterations(100);
        
        LevenbergMarquardtOptimizer optimizer(globalGraph, globalValues, params);
        Values result = optimizer.optimize();
        
        // Update optimized poses
        for (int i = 0; i < maxFrameId; ++i) {
            optimizedPoses[i] = result.at<Pose3>(Symbol('x', i));
        }
        
        ROS_INFO("Global optimization completed");
    }
    
    /**
     * Reconstruct global map after optimization (with lazy loading and memory management)
     */
    void reconstructGlobalMap()
    {
        ROS_INFO("Reconstructing global map after optimization (partial reconstruction)");
        
        globalMap->clear();
        
        // Only reconstruct up to current frame to avoid memory issues
        int maxFrameId = std::min(currentFrameId + 1, (int)pointCloudFiles.size());
        
        for (int i = 0; i < maxFrameId; ++i) {
            // Load frame on demand (cached)
            pcl::PointCloud<PointType>::Ptr frameCloud = getCachedPointCloud(i);
            if (!frameCloud) {
                ROS_WARN("Failed to load frame %d for global map reconstruction", i);
                continue;
            }
            
            // Filter frame to keep only points within 100m
            // filterPointCloudByDistance(frameCloud, 100.0);
            
            pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*frameCloud, *transformedCloud,
                                   optimizedPoses[i].matrix().cast<float>());
            
            *globalMap += *transformedCloud;
            
            // Progress update for large datasets
            if (i % 50 == 0) {
                ROS_INFO("Reconstructed %d/%d frames", i + 1, maxFrameId);
            }
        }

        // pcl::io::savePCDFileBinary( "/home/tyjt/Desktop/ros_ws/Reconstructed_global_map.pcd", *globalMap);
        
        // Final downsampling of global map
        // if (globalMap->size() > 10000) {
        //     pcl::PointCloud<PointType>::Ptr finalMap(new pcl::PointCloud<PointType>());
        //     downSizeFilterMap.setInputCloud(globalMap);
        //     downSizeFilterMap.filter(*finalMap);
        //     globalMap = finalMap;
        // }

        ROS_WARN("Saved Global map reconstructed with %d points from %d frames",
                 (int)globalMap->size(), maxFrameId);
    }
    
    /**
     * Save optimization results
     */
    void saveResults()
    {
        // Save optimized poses
        std::string posesFile = dataDirectory + "optimized_scan2map_poses.tum";
        std::ofstream file(posesFile);
        
        if (file.is_open()) {
            file << std::fixed << std::setprecision(6);
            for (int i = 0; i < optimizedPoses.size(); ++i) {
                Pose3 pose = optimizedPoses[i];
                Point3 t = pose.translation();
                Quaternion q = pose.rotation().toQuaternion();
                
                // Add back the offset when saving
                double tx_final = t.x() + tx_offset;
                double ty_final = t.y() + ty_offset;
                double tz_final = t.z() + tz_offset;

                // Use timestamp instead of frame index
                file << std::fixed << std::setprecision(3) << timestamps[i] << " " 
                     << std::setprecision(6) << tx_final << " " << ty_final << " " << tz_final << " "
                     << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            }
            file.close();
            ROS_INFO("Optimized poses saved to: %s (with offset restored: tx=%.6f, ty=%.6f, tz=%.6f)", 
                     posesFile.c_str(), tx_offset, ty_offset, tz_offset);
        }
        
        // Save global map
        std::string mapFile = dataDirectory +  "optimized_global_map.pcd";
        pcl::io::savePCDFileBinary(mapFile, *globalMap);
        ROS_INFO("Global map saved to: %s", mapFile.c_str());
        
        // Save loop closures
        std::string loopFile =  dataDirectory + "loop_closures.txt";
        std::ofstream loopStream(loopFile);
        if (loopStream.is_open()) {
            for (const auto& lc : loopClosures) {
                loopStream << lc.first << " " << lc.second << std::endl;
            }
            loopStream.close();
            ROS_INFO("Loop closures saved to: %s", loopFile.c_str());
        }
    }
    
    // Point association transformation
    Eigen::Affine3f transPointAssociateToMap;
};

/**
 * Main function
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2submap_optimizer_test");
    
    if (argc < 2) {
        ROS_ERROR("Usage: %s <data_directory>", argv[0]);
        return -1;
    }
    
    std::string dataDir = argv[1];
    
    Scan2MapOptimizer optimizer;
    
    // Load data
    if (!optimizer.loadData(dataDir)) {
        ROS_ERROR("Failed to load data from directory: %s", dataDir.c_str());
        return -1;
    }
    
    // Start optimization
    try {
        optimizer.optimize();
        ROS_INFO("Scan2Map optimization completed successfully!");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during optimization: %s", e.what());
        return -1;
    } catch (...) {
        ROS_ERROR("Unknown exception during optimization");
        return -1;
    }
    
    // Explicit cleanup before exit
    ROS_INFO("Program finishing, performing cleanup...");
    
    return 0;
}

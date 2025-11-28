/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3SLAMExample_g2o.cpp
 * @brief A 3D Pose SLAM example that reads input from g2o, and initializes the Pose3 using InitializePose3
 * Syntax for the script is ./Pose3SLAMExample_g2o input.g2o output.g2o [timestamps.tum]
 * The first pose is fixed during optimization. If timestamps.tum is provided, those timestamps will be used in output.
 * @date Aug 25, 2014
 * @author Luca Carlone
 */

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <ros/ros.h>

using namespace std;
using namespace gtsam;

// Function to apply trajectory smoothing
Values smoothTrajectory(const Values& input_poses, double smoothing_factor = 0.1) {
    cout << "Applying trajectory smoothing..." << endl;
    Values smoothed = input_poses;
    
    // Get sorted poses
    vector<pair<Key, Pose3>> sorted_poses;
    for (const auto& key_value : input_poses) {
        if (input_poses.exists<Pose3>(key_value.key)) {
            sorted_poses.push_back({key_value.key, input_poses.at<Pose3>(key_value.key)});
        }
    }
    sort(sorted_poses.begin(), sorted_poses.end(), 
         [](const pair<Key, Pose3>& a, const pair<Key, Pose3>& b) {
             return a.first < b.first;
         });
    
    if (sorted_poses.size() < 3) return smoothed;
    
    // Apply smoothing (skip first and last pose)
    for (size_t i = 1; i < sorted_poses.size() - 1; i++) {
        Pose3 prev = sorted_poses[i-1].second;
        Pose3 curr = sorted_poses[i].second;
        Pose3 next = sorted_poses[i+1].second;
        
        // Smooth translation
        Vector3 prev_trans = prev.translation();
        Vector3 curr_trans = curr.translation();
        Vector3 next_trans = next.translation();
        Vector3 smoothed_trans = curr_trans + smoothing_factor * 
            ((prev_trans + next_trans) / 2.0 - curr_trans);
        
        // Keep original rotation for now (rotation smoothing is more complex)
        Pose3 smoothed_pose(curr.rotation(), smoothed_trans);
        smoothed.update(sorted_poses[i].first, smoothed_pose);
    }
    
    cout << "Trajectory smoothing complete." << endl;
    return smoothed;
}

// Function to check trajectory consistency
void checkTrajectoryConsistency(const gtsam::Values& result) {
    std::vector<std::pair<uint64_t, gtsam::Pose3>> trajectory;
    
    // Collect all poses
    for (const auto& key_value : result) {
        if (gtsam::Symbol(key_value.key).chr() == 'x') {
            uint64_t key = key_value.key;
            gtsam::Pose3 pose = result.at<gtsam::Pose3>(key);
            trajectory.push_back(std::make_pair(key, pose));
        }
    }
    
    // Sort by key to ensure chronological order (sort only by key, not pose)
    std::sort(trajectory.begin(), trajectory.end(), 
              [](const std::pair<uint64_t, gtsam::Pose3>& a, const std::pair<uint64_t, gtsam::Pose3>& b) {
                  return a.first < b.first;
              });
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        gtsam::Pose3 prev = trajectory[i-1].second;
        gtsam::Pose3 curr = trajectory[i].second;
        
        // Calculate distance between translation vectors
        gtsam::Point3 diff = curr.translation() - prev.translation();
        double trans_step = diff.norm();
        
        // Check for large jumps that might cause ghosting
        if (trans_step > 10.0) {  // 10 meter threshold
            ROS_WARN("Large trajectory jump detected between pose %lu and %lu: %.2f meters", 
                     gtsam::Symbol(trajectory[i-1].first).index(), 
                     gtsam::Symbol(trajectory[i].first).index(), 
                     trans_step);
        }
    }
}

// Function to read timestamps from TUM file
map<Key, double> readTUMTimestamps(const string& tumFile) {
    map<Key, double> timestamps;
    ifstream file(tumFile);
    
    if (!file.is_open()) {
        cerr << "Warning: Cannot open TUM file for timestamps: " << tumFile << endl;
        return timestamps;
    }
    
    cout << "Reading timestamps from TUM file: " << tumFile << endl;
    
    string line;
    Key currentKey = 0;
    
    while (getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        
        if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
            timestamps[currentKey] = timestamp;
            currentKey++;
        }
    }
    
    file.close();
    cout << "Read " << timestamps.size() << " timestamps from TUM file" << endl;
    return timestamps;
}

// Function to save trajectory in TUM format with optional timestamps
bool saveTUMTrajectory(const Values& values, const string& filename, const map<Key, double>& timestamps = map<Key, double>()) {
    cout << "Saving trajectory in TUM format to: " << filename << endl;
    
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open file: " << filename << endl;
        return false;
    }
    
    // Write TUM format header
    // file << "# TUM trajectory format" << endl;
    // file << "# timestamp tx ty tz qx qy qz qw" << endl;
    
    // Sort poses by key for consistent output
    map<Key, Pose3> sorted_poses;
    for (const auto& key_value : values) {
        if (values.exists<Pose3>(key_value.key)) {
            sorted_poses[key_value.key] = values.at<Pose3>(key_value.key);
        }
    }
    
    cout << "Writing " << sorted_poses.size() << " poses to TUM file..." << endl;
    bool using_external_timestamps = !timestamps.empty();
    if (using_external_timestamps) {
        cout << "Using external timestamps from TUM file" << endl;
    } else {
        cout << "Using key values as timestamps" << endl;
    }
    
    for (const auto& pair : sorted_poses) {
        const Pose3& pose = pair.second;
        Vector3 translation = pose.translation();
        gtsam::Quaternion rotation = pose.rotation().toQuaternion();
        
        // Use external timestamp if available, otherwise use key as timestamp
        double timestamp;
        if (using_external_timestamps && timestamps.find(pair.first) != timestamps.end()) {
            timestamp = timestamps.at(pair.first);
        } else {
            timestamp = static_cast<double>(pair.first);
        }
        
        // Write in TUM format: timestamp tx ty tz qx qy qz qw
        file << fixed << setprecision(3)
             << timestamp << " "
             << setprecision(6) 
             << translation.x() << " " << translation.y() << " " << translation.z() << " "
             << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w()
             << endl;
    }
    
    file.close();
    cout << "Successfully saved TUM trajectory with " << sorted_poses.size() << " poses!" << endl;
    return true;
}

int main(int argc, char** argv) {
  // Initialize ROS for parameter reading
  ros::init(argc, argv, "pose3_slam_g2o");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

    // 获取参数
    std::string input_g2o_file, output_g2o_file, output_tum_file,input_tum_file;
    std::string optimization_method;
    std::string work_dir = "/mnt/nvme0n1p2/data/28_3";

    pnh.param<std::string>("input_g2o_file", input_g2o_file, work_dir + "/pose_graph.g2o");
    pnh.param<std::string>("output_g2o_file", output_g2o_file, work_dir + "/graph_opt.g2o");

    pnh.param<std::string>("input_tum_file", input_tum_file, work_dir + "/geo_key_pose_opt.tum");
    pnh.param<std::string>("output_tum_file", output_tum_file, work_dir + "/graph_opt_trajectory.tum");
    pnh.param<std::string>("optimization_method", optimization_method, "ISAM2"); // "LM" or "ISAM2"

  // Check arguments
    std::cout << "=== G2O File Optimizer ===" << std::endl;
    std::cout << "Input g2o file: " << input_g2o_file << std::endl;
    std::cout << "Output g2o file: " << output_g2o_file << std::endl;
    std::cout << "input_tum_file TUM file: " << input_tum_file << std::endl;
    std::cout << "Output TUM file: " << output_tum_file << std::endl;
    std::cout << "Optimization method: " << optimization_method << std::endl;
    std::cout << "==========================\n" << std::endl;

  // Read optimization parameters
  int max_iterations;
  double relative_error_tol, absolute_error_tol, smoothing_factor;
  std::string verbosity;
  bool enable_smoothing, enable_multi_stage;
  
  pnh.param<int>("max_iterations", max_iterations, 200);
  pnh.param<double>("relative_error_tol", relative_error_tol, 1e-5);
  pnh.param<double>("absolute_error_tol", absolute_error_tol, 1e-3);
  pnh.param<std::string>("verbosity", verbosity, "TERMINATION");
  pnh.param<bool>("enable_smoothing", enable_smoothing, true);
  pnh.param<bool>("enable_multi_stage", enable_multi_stage, true);
  pnh.param<double>("smoothing_factor", smoothing_factor, 0.1);

  cout << "=== Optimization Parameters ===" << endl;
  cout << "Max iterations: " << max_iterations << endl;
  cout << "Relative error tolerance: " << relative_error_tol << endl;
  cout << "Absolute error tolerance: " << absolute_error_tol << endl;
  cout << "Verbosity: " << verbosity << endl;
  cout << "Multi-stage optimization: " << (enable_multi_stage ? "ON" : "OFF") << endl;
  cout << "Trajectory smoothing: " << (enable_smoothing ? "ON" : "OFF") << endl;
  if (enable_smoothing) {
    cout << "Smoothing factor: " << smoothing_factor << endl;
  }
  cout << "===============================" << endl;
  

  // Read graph from file

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = true;
  boost::tie(graph, initial) = readG2o(input_g2o_file, is3D);

  // Add strong prior on the first key to prevent it from moving during optimization
  // Using very small variances to effectively fix the first pose
  auto priorModel = noiseModel::Diagonal::Variances(
      (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
  Key firstKey = 0;
  Pose3 firstPose;
  for (const auto key_value : *initial) {
    std::cout << "Adding strong prior to first pose (key=" << key_value.key << ") to fix it during optimization" << std::endl;
    firstKey = key_value.key;
    firstPose = key_value.value.cast<Pose3>();
    // Use the actual initial pose instead of identity
    graph->addPrior(firstKey, firstPose, priorModel);
    std::cout << "First pose fixed at: " << std::endl;
    firstPose.print("  First Pose: ");
    break;
  }

  Values result;
  
  if (enable_multi_stage) {
    std::cout << "=== Multi-Stage Optimization for High Precision ===" << std::endl;
    
    // Stage 1: Initial optimization with relaxed settings
    std::cout << "Stage 1: Initial optimization..." << std::endl;
    GaussNewtonParams params1;
    params1.setVerbosity("TERMINATION");
    params1.setMaxIterations(max_iterations / 2);
    params1.setRelativeErrorTol(relative_error_tol );
    params1.setAbsoluteErrorTol(absolute_error_tol );
    
    GaussNewtonOptimizer optimizer1(*graph, *initial, params1);
    Values result1 = optimizer1.optimize();
    std::cout << "Stage 1 error: " << graph->error(result1) << std::endl;
    
    // Stage 2: Fine-tuning with strict settings
    std::cout << "Stage 2: Fine-tuning optimization..." << std::endl;
    GaussNewtonParams params2;
    params2.setVerbosity(verbosity);
    params2.setMaxIterations(max_iterations);
    params2.setRelativeErrorTol(relative_error_tol);
    params2.setAbsoluteErrorTol(absolute_error_tol);
    
    GaussNewtonOptimizer optimizer2(*graph, result1, params2);
    Values result2 = optimizer2.optimize();
    std::cout << "Stage 2 error: " << graph->error(result2) << std::endl;
    
    // Stage 3: Ultra-fine optimization for precision
    std::cout << "Stage 3: Ultra-precision optimization..." << std::endl;
    GaussNewtonParams params3;
    params3.setVerbosity("ERROR");
    params3.setMaxIterations(max_iterations / 2);
    params3.setRelativeErrorTol(relative_error_tol / 10);
    params3.setAbsoluteErrorTol(absolute_error_tol / 10);
    
    GaussNewtonOptimizer optimizer3(*graph, result2, params3);
    result = optimizer3.optimize();
    std::cout << "=== Multi-Stage Optimization Complete ===" << std::endl;
  } else {
    std::cout << "=== Single-Stage Optimization ===" << std::endl;
    GaussNewtonParams params;
    params.setVerbosity(verbosity);
    params.setMaxIterations(max_iterations);
    params.setRelativeErrorTol(relative_error_tol);
    params.setAbsoluteErrorTol(absolute_error_tol);
    
    GaussNewtonOptimizer optimizer(*graph, *initial, params);
    result = optimizer.optimize();
    std::cout << "=== Single-Stage Optimization Complete ===" << std::endl;
  }
  
  std::cout << "Final optimization error: " << graph->error(result) << std::endl;

  std::cout << "initial error=" << graph->error(*initial) << std::endl;
  std::cout << "final error=" << graph->error(result) << std::endl;
  
  // Apply post-processing
//   checkTrajectoryConsistency(result);
  
  if (0) {
//   if (enable_smoothing) {
    Values smoothed_result = smoothTrajectory(result, smoothing_factor);
    std::cout << "After smoothing error=" << graph->error(smoothed_result) << std::endl;
    result = smoothed_result;
  }

  // Verify first pose hasn't moved
  if (result.exists(firstKey)) {
    Pose3 optimizedFirstPose = result.at<Pose3>(firstKey);
    std::cout << "\n=== First Pose Verification ===" << std::endl;
    std::cout << "Original first pose:" << std::endl;
    firstPose.print("  Original: ");
    std::cout << "Optimized first pose:" << std::endl;
    optimizedFirstPose.print("  Optimized: ");
    
    // Check if they are approximately equal
    double translation_diff = (optimizedFirstPose.translation() - firstPose.translation()).norm();
    double rotation_diff = optimizedFirstPose.rotation().between(firstPose.rotation()).matrix().trace();
    std::cout << "Translation difference: " << translation_diff << std::endl;
    std::cout << "Rotation trace difference: " << rotation_diff << std::endl;
    std::cout << "First pose " << (translation_diff < 1e-10 ? "successfully fixed" : "moved during optimization") << std::endl;
    std::cout << "==============================\n" << std::endl;
 
    std::cout << "Writing results to file: " << output_g2o_file << std::endl;
    
    // Write the complete graph (including constraints) with optimized values
    std::cout << "Including " << graph->size() << " factors (constraints + priors) in output" << std::endl;
    writeG2o(*graph, result, output_g2o_file);
    std::cout << "Successfully written optimized graph with all constraints to: " << output_g2o_file << std::endl;
    
    // Read timestamps from TUM file if provided
    map<Key, double> external_timestamps;
    std::cout << "Reading timestamps from: " << input_tum_file << std::endl;
    external_timestamps = readTUMTimestamps(input_tum_file);

    // Save TUM trajectory with timestamps
    string tumFile = output_g2o_file.substr(0, output_g2o_file.find_last_of('.')) + "_trajectory.tum";
    if (saveTUMTrajectory(result, tumFile, external_timestamps)) {
        std::cout << "TUM trajectory saved to: " << tumFile << std::endl;
        if (!external_timestamps.empty()) {
            std::cout << "Used " << external_timestamps.size() << " external timestamps" << std::endl;
        }
    } else {
        std::cerr << "Failed to save TUM trajectory!" << std::endl;
    }
  }
  return 0;
}

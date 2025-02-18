cmake_minimum_required(VERSION 3.0.2)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -std=c++17 -fexceptions")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  visualization_msgs
  # tf
)

# set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  sensor_msgs
  image_transport
  message_generation
  cmake_modules
  dynamic_reconfigure
  livox_ros_driver
)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

# # Hierarchical Bundle Adjustment
add_executable(hba source/hba.cpp)
target_link_libraries(hba ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(visualize_map source/visualize.cpp)
target_link_libraries(visualize_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# reproject_pts_img with odometry
add_executable(reproject_pts_img source/project.cpp)
target_link_libraries(reproject_pts_img ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(reproject_img_pts_with_pose source/reproject_img_pts.cpp)
target_link_libraries(reproject_img_pts_with_pose ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(project_points2img_offine source/project_points2img_offine.cpp)
target_link_libraries(project_points2img_offine ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(odom2posegraph source/odom2posegraph.cpp)
target_link_libraries(odom2posegraph ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(get_camera_pose source/get_camera_pose.cpp)
target_link_libraries(get_camera_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(shadow_filter source/shadow_filter.cpp)
target_link_libraries(shadow_filter ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(get_lidar_intensity_image source/get_lidar_intensity_image.cpp)
target_link_libraries(get_lidar_intensity_image ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} )


add_executable(test_opencv source/test_opencv.cpp)
target_link_libraries(test_opencv ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} )

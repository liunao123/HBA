cmake_minimum_required(VERSION 3.0.2)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -std=c++17 -fexceptions")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++1z -std=c++17 -fexceptions")

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
  # sudo apt install -f ros-noetic-livox-ros-driver
  livox_ros_driver
)
find_package(OpenCV REQUIRED)

# Force PCL 1.15 from /usr/local to avoid mixing with system PCL 1.12.
if (EXISTS "/usr/local/share/pcl-1.15/PCLConfig.cmake")
  set(PCL_DIR "/usr/local/share/pcl-1.15" CACHE PATH "PCL CMake config directory" FORCE)
endif()


# 02_Pose3SLAMExample_g2o 需要PCL 1.15，否则编译没问题，运行会崩溃
find_package(PCL 1.15 REQUIRED) 
# find_package(PCL  REQUIRED) 


# find_package(PCL REQUIRED)
message(STATUS "PCL_DIR: ${PCL_DIR}")
message(STATUS "PCL version: ${PCL_VERSION}")

# Removed explicit find_package(chcnav). We only need its generated headers; building the chcnav package will place them in ${CATKIN_DEVEL_PREFIX}/include.
# find_package(GeographicLib REQUIRED)

# 需要卸载系统自带的3.4，源码安装3.3.7 copy到系统路径下再编译
# sudo cp -r /path/to/eigen/Eigen /usr/include/eigen3/
# sudo cp -r /path/to/eigen/unsupported /usr/include/eigen3/
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# oneTBB 在M-detector里已经有了 这里就不需要再下载一份了
# set(TBB_INCLUDE_DIR "/home/xf/Desktop/catkin_ws/src/M-detector/oneTBB-2019_U8/include")
# set(TBB_LIBRARY_DIR "/home/xf/Desktop/catkin_ws/src/M-detector/oneTBB-2019_U8/build/linux_intel64_gcc_cc11.4.0_libc2.35_kernel6.8.0_release")
# include_directories(${TBB_INCLUDE_DIR})
# link_directories(${TBB_LIBRARY_DIR})
# find_package(TBB REQUIRED)

# 如果系统中已经安装了TBB，可以直接使用以下方式指定 ，否则需要手动指定库和头文件路径。
set(TBB_LIBRARIES /usr/local/lib/libtbb.so /usr/local/lib/libtbbmalloc.so)
set(TBB_INCLUDE_DIRS /usr/local/tbb-2019_U8/include)
include_directories(${TBB_INCLUDE_DIRS})

# 源码编译的gtsam 4.1.1
# 设置GTSAM的路径
set(gtsam_DIR "/home/xf/Documents/gtsam-4.1.1/build/")
# 或者直接设置库和头文件路径
set(GTSAM_LIBRARY "/home/xf/Documents/gtsam-4.1.1/build/gtsam/libgtsam.so")
set(GTSAM_INCLUDE_DIR "/home/xf/Documents/gtsam-4.1.1/build/gtsam")

find_package(GTSAM REQUIRED)
if(GTSAM_FOUND)
    message(STATUS "Found GTSAM ${GTSAM_VERSION}")
else()
    message(STATUS "GTSAM Not Found")
endif()
message(STATUS "GTSAM_DIR=${GTSAM_DIR}")
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)
# Ensure GDAL dependencies are resolved
find_package(GDAL REQUIRED)
# Add CURL, TIFF, and GDAL libraries to Pose3SLAMExample_g2o
set(GDAL_LIBRARIES ${GDAL_LIBRARIES} ${CURL_LIBRARIES} ${TIFF_LIBRARIES})


include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  nano_gicp/include
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_library(nano_gicp_lib
  include/nano_gicp/nanoflann.cc
  include/nano_gicp/lsq_registration.cc
  include/nano_gicp/nano_gicp.cc
)

target_link_libraries(nano_gicp_lib
  ${PCL_LIBRARIES}
  ${OPENMP_LIBS}
)

target_include_directories(nano_gicp_lib PUBLIC
  nano_gicp/include
  ${PCL_INCLUDE_DIRS}
)

add_executable(01_get_raw_data source/01_get_raw_data.cpp)
target_link_libraries(01_get_raw_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GDAL_LIBRARIES}
  Geographic
  yaml-cpp
)


set(FAST_GICP_CUDA_LIB_DIR "/home/xf/Documents/fast_gicp/build/devel/lib")
set(PATCHWORKPP_INCLUDE_DIR "/home/xf/Documents/patchwork-plusplus/cpp/patchworkpp/include")
set(PATCHWORKPP_SOURCE "/home/xf/Documents/patchwork-plusplus/cpp/patchworkpp/src/patchworkpp.cpp")

include_directories(${PATCHWORKPP_INCLUDE_DIR})

add_executable(02_Pose3SLAMExample_g2o
  source/02_Pose3SLAMExample_g2o.cpp
)

target_link_libraries(02_Pose3SLAMExample_g2o
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}  
  ${LZ4_LIB}
  ${GDAL_LIBRARIES}
  ${TBB_LIBRARIES}
  nano_gicp_lib
  ${FAST_GICP_CUDA_LIB_DIR}/libfast_gicp.so
  ${FAST_GICP_CUDA_LIB_DIR}/libfast_vgicp_cuda.so
  gtsam
  yaml-cpp
  Geographic
)
set_target_properties(02_Pose3SLAMExample_g2o PROPERTIES
  BUILD_RPATH "${FAST_GICP_CUDA_LIB_DIR}"
)

add_executable(02_refined_ground
  source/02_refined_ground.cpp
  ${PATCHWORKPP_SOURCE}
)

target_link_libraries(02_refined_ground
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}  
  nano_gicp_lib
  yaml-cpp
  gtsam
  Geographic
)


add_executable(pub_pose_and_pts source/pub_pose_and_pts.cpp)
target_link_libraries(pub_pose_and_pts ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
  yaml-cpp
  Geographic
)

add_executable(visualize_with_pose_pcd source/visualize_with_pose_pcd.cpp)
target_link_libraries(visualize_with_pose_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# # # Hierarchical Bundle Adjustment
# add_executable(hba source/hba.cpp)
## target_link_libraries(hba ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam Eigen3::Eigen  TBB::tbb)
# target_link_directories(hba PRIVATE
#   "/home/tyjt/Downloads/gtsam/build/gtsam" # 包含 libgtsam.so 的路径
# )

# target_include_directories(hba PRIVATE
#   "/home/username/gtsam" # 包含 gtsam/ 目录的路径
# )



# # # 读取graph.g2o里面的位姿
# add_executable(visualize_map source/visualize.cpp)
# target_link_libraries(visualize_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# # # 去读取和pcd文件在同一个地方的data文件里面的位姿
# add_executable(visualize_without_g2o source/visualize_without_g2o.cpp)
# target_link_libraries(visualize_without_g2o ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# # # reproject_pts_img with odometry
# # # pose is must
# add_executable(reproject_pts_img source/project.cpp)
# target_link_libraries(reproject_pts_img ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

# # # pose is NOt must . just reproject pts to img if not pose
# add_executable(reproject_img_pts_with_pose source/reproject_img_pts_with_pose.cpp)
# target_link_libraries(reproject_img_pts_with_pose ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

# # 离线读取img和pcd进行投影 可以确认外参是不是正确
# add_executable(project_points2img_offine source/project_points2img_offine.cpp)
# target_link_libraries(project_points2img_offine ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} )

# # get camera pose for 3dgs
# add_executable(get_camera_pose source/get_camera_pose.cpp)
# target_link_libraries(get_camera_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# # 异常点去除逻辑 目前只使用与livox固态雷达
# add_executable(livox_to_pcd_filter source/livox_to_pcd_filter.cpp)
# target_link_libraries(livox_to_pcd_filter ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

# # 去除噪点的pcd根据此来生成强度图，然后提取出线特征
# add_executable(get_lidar_intensity_image source/get_lidar_intensity_image.cpp)
# target_link_libraries(get_lidar_intensity_image ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES}  Eigen3::Eigen )

# # 把关键帧写成文件 这个是将所有的帧保存下来
# add_executable(odom2posegraph source/odom2posegraph.cpp)
# target_link_libraries(odom2posegraph ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

# # 这个按距离阈值保存
# add_executable(odom2posegraph_keyframe source/odom2posegraph_keyframe.cpp)
# target_link_libraries(odom2posegraph_keyframe ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

# Add executable for testing SensorExtrinsics
add_executable(test_sensor_extrinsics_ros source/sensor_extrinsics_ros.cpp)
target_link_libraries(test_sensor_extrinsics_ros
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

# add_executable(get_session_data_and_NonUniformSampling source/get_session_data_and_NonUniformSampling.cpp)
# target_link_libraries(get_session_data_and_NonUniformSampling
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${GDAL_LIBRARIES}
#   Eigen3::Eigen
#   yaml-cpp
#   Geographic
# )

# pcl1.15 只能有一个版本
add_executable(03_GetRgbAndNonuniformSample source/03_GetRgbAndNonuniformSample.cpp)
target_link_libraries(03_GetRgbAndNonuniformSample
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GDAL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
  Geographic
)
target_link_directories(03_GetRgbAndNonuniformSample PRIVATE /usr/local/lib)
set_target_properties(03_GetRgbAndNonuniformSample PROPERTIES
  BUILD_RPATH "/usr/local/lib"
  INSTALL_RPATH "/usr/local/lib"
)

# Ensure the include directories are added
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)


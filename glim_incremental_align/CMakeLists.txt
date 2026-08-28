cmake_minimum_required(VERSION 3.22)
project(local_align LANGUAGES CXX)

# =============================================================================
# 独立项目: 与 glim 无关。
#
# 原来这份代码住在 glim/incremental_align/ 里, 链 libglim.so。实测 glim 那边真正被用到的
# 只有 CloudCovarianceEstimation 一个类(292 行), 已整文件搬进 src/, 于是:
#   - 不再需要 libglim.so, 也不需要先编译 glim
#   - 不再需要 GLIM_USE_OPENCV 宏 (那是为了和 libglim.so 的虚表布局一致; 不链它就无所谓)
#   - 不再需要 glim 的 config/ 目录 (logging/sensors/global_mapping 那几个 json 只被
#     glim 的 GlobalMapping/IMUIntegration 读, 本程序两个都不用)
#
# 剩下的外部依赖: Eigen / GTSAM / gtsam_points / OpenCV / Boost / OpenMP / Ceres /
#                 yaml-cpp / zlib / TBB
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# 依赖前缀。**必须在 find_package 之前** ——
# 本机 /usr/local 里还装着一个旧的 GTSAM 4.1.1, 而要用的是 4.3a0 的**build tree**
# (GTSAM 支持直接从 build tree 使用, 那里有 GTSAMConfig.cmake)。不写这段的话
# find_package 只能从默认路径搜, 命中 4.1.1 然后报 "not compatible with version 4.2"。
# ---------------------------------------------------------------------------
set(GTSAM_BUILD_DIR "/mnt/nvme0n1p2/gtsam_map/gtsam/build" CACHE PATH "GTSAM 4.3a0 build tree")
set(GTSAM_POINTS_PREFIX "/mnt/nvme0n1p2/gtsam_map/install/gtsam_points-1.2.2" CACHE PATH "gtsam_points install prefix")
list(PREPEND CMAKE_PREFIX_PATH "${GTSAM_BUILD_DIR}" "${GTSAM_POINTS_PREFIX}")

find_package(Eigen3 REQUIRED)
find_package(GTSAM 4.2 REQUIRED)
find_package(gtsam_points 1.2.2 REQUIRED)
# 视觉重投影约束需要 features2d(ORB)/calib3d(基础矩阵)/imgcodecs(读图)/imgproc(颜色空间)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
find_package(Boost REQUIRED COMPONENTS serialization filesystem)
find_package(OpenMP REQUIRED)
find_package(Ceres REQUIRED)          # submap 内部 BA (submap_ceres.hpp)
find_package(yaml-cpp REQUIRED)       # 全部参数由 yaml 指定 (--params), 见 params/*.yaml
# zlib: src/png_io.hpp 用它压 PNG 的 IDAT。必须显式链 —— libz 虽被其他依赖间接拉进来,
# 但 crc32/compress2 是我们自己直接调的, 不写会报 "DSO missing from command line"。
find_package(ZLIB REQUIRED)
# nlohmann/json: session_loader.hpp 读 manifest / egomotions / 3dod 标注
find_package(nlohmann_json QUIET)

# ---------------------------------------------------------------------------
# TBB: 本机 GTSAM 4.3a0 是 GTSAM_WITH_TBB=ON + GTSAM_DEFAULT_ALLOCATOR=TBB 编的,
# 于是它的头文件把 NonlinearFactorGraph 的底层容器换成了 tbb::tbb_allocator,
# 可执行文件必须自己链 TBB。
#
# 坑: 本机同时装了两套 ABI ——
#   libtbb.so.12 (oneTBB, 也是 libtbb.so 的指向)  -> 没有 deallocate_via_handler_v3 等符号
#   libtbb.so.2  (TBB 2020 旧 ABI)               -> GTSAM/gtsam_points 实际需要的这套
# 所以不能简单写 -ltbb, 必须显式指到 .so.2。
# ---------------------------------------------------------------------------
find_library(TBB_OLD_LIB NAMES libtbb.so.2
  PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib NO_DEFAULT_PATH)
find_library(TBBMALLOC_LIB NAMES libtbbmalloc.so.2
  PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib NO_DEFAULT_PATH)
if(NOT TBB_OLD_LIB)
  message(FATAL_ERROR "找不到 libtbb.so.2 (GTSAM 用的是旧 TBB ABI, libtbb.so.12 不兼容)")
endif()

message(STATUS "GTSAM_VERSION        = ${GTSAM_VERSION}")
message(STATUS "gtsam_points_VERSION = ${gtsam_points_VERSION}")
message(STATUS "OpenCV_VERSION       = ${OpenCV_VERSION}")
message(STATUS "TBB                  = ${TBB_OLD_LIB}")

# ---------------------------------------------------------------------------
# 运行时库搜索路径
#
# 坑: /usr/local 里装着一个旧的 GTSAM 4.1.1 (libgtsam.so.4)。CMake 默认生成 DT_RUNPATH,
# 而 RUNPATH **不传递** —— libgtsam_points.so 自己没有 RUNPATH, 加载器解析它的
# libgtsam.so.4 依赖时不会参考可执行文件的 RUNPATH, 于是退回 ldconfig 缓存, 命中 4.1.1,
# 表现为运行期 "undefined symbol: gtsam::NonlinearFactor::error(HybridValues)"。
# 用 --disable-new-dtags 改生成 DT_RPATH (可传递), 并把本地 4.3a0 放在最前。
# ---------------------------------------------------------------------------
set(RUNTIME_LIB_DIRS
  "${GTSAM_BUILD_DIR}/gtsam"
  "${GTSAM_BUILD_DIR}/gtsam_unstable"
  "${GTSAM_POINTS_PREFIX}/lib"
)

add_executable(local_align
  src/local_align.cpp
  src/cloud_covariance_estimation.cpp   # 从 glim 搬过来的, 见该文件头部说明
)
target_include_directories(local_align PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(local_align PRIVATE
  gtsam_points::gtsam_points
  gtsam
  Eigen3::Eigen
  Boost::serialization
  Boost::filesystem
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
  Ceres::ceres
  yaml-cpp
  ZLIB::ZLIB
  ${TBB_OLD_LIB}
  ${TBBMALLOC_LIB}
)
if(nlohmann_json_FOUND)
  target_link_libraries(local_align PRIVATE nlohmann_json::nlohmann_json)
endif()
target_link_options(local_align PRIVATE "-Wl,--disable-new-dtags")
set_target_properties(local_align PROPERTIES
  BUILD_RPATH "${RUNTIME_LIB_DIRS}" INSTALL_RPATH "${RUNTIME_LIB_DIRS}")

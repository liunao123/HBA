#pragma once

#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <thread>

// OpenMP
#include <omp.h>

// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

// BOOST
#include <boost/format.hpp>

// eigen
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

// opencv
// #include <opencv2/opencv.hpp>

const double gravity_ = 9.80665;

const double rad2deg = 180.0 / M_PI;
const double deg2rad = M_PI / 180.0;

struct Point {
    PCL_ADD_POINT4D;
    uint8_t  intensity;
    double   timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(Point, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
                                             double, timestamp, timestamp)(uint16_t, ring, ring))

                                             

typedef Point PointType;

// Hcinspvatzcb
struct chcnav_devpvt {
    double          timestamp;
    double          latitude; //设备(ins)的经纬高
    double          longitude;
    double          altitude;
    Eigen::Vector3f position_stdev;
    float           undulation;

    float           roll;
    float           pitch;
    float           yaw;
    Eigen::Vector3f euler_stdev;

    float speed;    //地面速度
    float heading;  //速度航向，正北为0，顺时针为正，0°~360°
    float heading2; //双天线航向，正北为0，顺时针为正，0°~360°

    Eigen::Vector3f enu_velocity; // ENU坐标系下速度
    Eigen::Vector3f enu_velocity_stdev;

    Eigen::Vector3f vehicle_angular_velocity;              //车辆坐标系下，去零偏 deg/s
    Eigen::Vector3f vehicle_linear_velocity;               //车辆坐标系下速度
    Eigen::Vector3f vehicle_linear_acceleration;           // 车辆坐标系下，去零偏，不补偿重力
    Eigen::Vector3f vehicle_linear_acceleration_without_g; // 车辆坐标系下，去零偏，补偿重力

    // 与/chcnav/imu_raw topic一致 注意：/chcnav/imu_raw中单位为rad/s , m/s^2
    Eigen::Vector3f raw_angular_velocity; //设备坐标系下，不去零偏, 单位:deg/s
    Eigen::Vector3f raw_acceleration;     //设备坐标系下，不去零偏，不补偿重力, 单位:g
};

struct ImuMeas {
    double stamp;
    double dt;                 // defined as the difference between the current and the previous
                               // measurement
    Eigen::Vector3f ang_vel;   // rad/s
    Eigen::Vector3f lin_accel; // m/s^2
};

struct Odometry {
    double          timestamp;
    Eigen::Vector3d position_lla; // in LLA system
    Eigen::Matrix3d pose;         // in ENU system
    Eigen::Vector3d enu_velocity;
    double          heading;
    double          speed;
    ImuMeas         imu_raw;
    ImuMeas         imu_meas;
};

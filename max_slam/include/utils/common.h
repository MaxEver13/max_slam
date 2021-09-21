/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-16 15:28:01
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-19 23:06:42
 */

#ifndef SRC_MAX_SLAM_INCLUDE_UTILS_COMMON_H_
#define SRC_MAX_SLAM_INCLUDE_UTILS_COMMON_H_


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZI PointType;

// 公共参数解析
class ParamServer
{
public:
    // ros句柄　全局空间解析参数
    ros::NodeHandle nh_;

    // 订阅的Topics
    string point_cloud_topic_;
    string imu_topic_;
    string odom_topic_;
    string gps_topic_;

    // 坐标系
    string lidar_frame_;
    string baselink_frame_;
    string odometry_frame_;
    string map_frame_;

    // Lidar Params
    int N_SCAN;
    int HORIZON_SCAN;
    float LIDAR_MIN_RANGE;
    float LIDAR_MAX_RANGE;

    // IMU Params
    float imu_acc_noise_;          // 加速度噪声标准差
    float imu_gyr_noise_;          // 角速度噪声标准差
    float imu_acc_biasN_;          // 加速度bias factor之间的约束噪声协方差参数
    float imu_gyr_biasN_;          // 陀螺仪bias factor之间的约束噪声协方差参数
    float imu_gravity_;            // 重力加速度

    // LIDAR IMU EXTRINSICS
    std::vector<double> ext_rot_v_;
    std::vector<double> ext_rpy_v_;
    std::vector<double> ext_trans_v_;
    Eigen::Matrix3d ext_rot_;     // lidar与imu外参旋转：旋转矩阵表示
    Eigen::Matrix3d ext_rpy_;     // lidar与imu外参旋转：RPY欧拉角表示
    Eigen::Vector3d ext_trans_;   // lidar与imu外参平移向量
    Eigen::Quaterniond ext_q_;    // lidar与imu外参旋转：四元数表示  



    ParamServer()
    {
        nh_.param<std::string>("pointCloudTopic", point_cloud_topic_, "points_raw");
        nh_.param<std::string>("imuTopic", imu_topic_, "imu_correct");
        nh_.param<std::string>("odomTopic", odom_topic_, "odometry/imu");
        nh_.param<std::string>("gpsTopic", gps_topic_, "odometry/gps");

        nh_.param<std::string>("lidarFrame", lidar_frame_, "base_link");
        nh_.param<std::string>("baselinkFrame", baselink_frame_, "base_link");
        nh_.param<std::string>("odometryFrame", odometry_frame_, "odom");
        nh_.param<std::string>("mapFrame", map_frame_, "map");

        nh_.param<int>("N_SCAN", N_SCAN, 64);
        nh_.param<int>("Horizon_SCAN", HORIZON_SCAN, 1800);
        nh_.param<float>("lidarMinRange", LIDAR_MIN_RANGE, 1.0);
        nh_.param<float>("lidarMaxRange", LIDAR_MAX_RANGE, 1000.0);

        nh_.param<float>("imuAccNoise", imu_acc_noise_, 0.01);
        nh_.param<float>("imuGyrNoise", imu_gyr_noise_, 0.001);
        nh_.param<float>("imuAccBiasN", imu_acc_biasN_, 0.0002);
        nh_.param<float>("imuGyrBiasN", imu_gyr_biasN_, 0.00003);
        nh_.param<float>("imuGravity", imu_gravity_, 9.80511);
        nh_.param<vector<double>>("extrinsicRot", ext_rot_v_, vector<double>());
        nh_.param<vector<double>>("extrinsicRPY", ext_rpy_v_, vector<double>());
        nh_.param<vector<double>>("extrinsicTrans", ext_trans_v_, vector<double>());
        ext_rot_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_rot_v_.data(), 3, 3);
        ext_rpy_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_rpy_v_.data(), 3, 3);
        ext_trans_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_trans_v_.data(), 3, 1);
        ext_q_ = Eigen::Quaterniond(ext_rpy_);

        usleep(100);
    }


    /**
     * @function: 将imu原始数据转换到lidar坐标系
     * @param {sensor_msgs::Imu}
     * @return {sensor_msgs::Imu}
     */
    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in) 
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = ext_rot_ * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = ext_rot_ * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        // Eigen四元数是Hamilton convention 需要右乘四元数
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * ext_q_;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

#endif //SRC_MAX_SLAM_INCLUDE_UTILS_COMMON_H_

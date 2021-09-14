//
// Created by xchu on 2021/5/16.
// Modifier: Jiawen Ji 
//
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

typedef pcl::PointXYZI PointT;

inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Eigen::MatrixXd cov;

  Pose6D() {
    x = y = z = roll = pitch = yaw = 0.0;
    cov.setIdentity();
  };

  Pose6D(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
      : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw) {
    cov.setIdentity();
  };
};

Pose6D operator+(const Pose6D &A, const Pose6D B) //给结构体定义加法；
{
  return Pose6D{A.x + B.x, A.y + B.y, A.z + B.z, A.roll + B.roll, A.pitch + B.pitch, A.yaw + B.yaw};
}

Pose6D operator-(const Pose6D &A, const Pose6D B) {
  return Pose6D{A.x - B.x, A.y - B.y, A.z - B.z, A.roll - B.roll, A.pitch - B.pitch, A.yaw - B.yaw};
}

std::ostream &operator<<(std::ostream &out, const Pose6D &p)  //定义结构体流输出
{
  out << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << "," << p.yaw << ")";
  return out;
}
extern Pose6D Matrix2Pose6D(const Eigen::Matrix4d &matrix) {
  Eigen::Vector3d pos = matrix.block<3, 1>(0, 3).matrix();
  Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).matrix();
  Eigen::Quaterniond quat(rot);

  Pose6D p;
  p.x = pos(0);
  p.y = pos(1);
  p.z = pos(2);
  tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(p.roll, p.pitch, p.yaw);
  return p;
}

extern Eigen::Isometry3d Matrix2Isometry3d(const Eigen::Matrix4d &matrix) {
  Eigen::Vector3d pos = matrix.block<3, 1>(0, 3).matrix();
  Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0).matrix();
  Eigen::Quaterniond quat(rot);

  Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
  pose_eigen.rotate(rot);
  pose_eigen.pretranslate(pos);

  return pose_eigen;
}

extern Pose6D Isometry3d2Pose6D(const Eigen::Isometry3d &matrix) {
  Eigen::Matrix3d rot = matrix.rotation().matrix();
  Eigen::Quaterniond quat(rot);

  Pose6D p;
  p.x = matrix.translation().x();
  p.y = matrix.translation().y();
  p.z = matrix.translation().z();
  tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(p.roll, p.pitch, p.yaw);
  return p;
}

extern Eigen::Matrix4d Pose6D2Matrix(const Pose6D p) {
  Eigen::Translation3d tf_trans(p.x, p.y, p.z);
  Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix4d mat = (tf_trans * rot_z * rot_y * rot_x).matrix();
  return mat;
}

extern Eigen::Isometry3d GeometryToEigen(const nav_msgs::Odometry &pose_geo) {
  Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
  pose_eigen.rotate(Eigen::Quaterniond(pose_geo.pose.pose.orientation.w,
                                       pose_geo.pose.pose.orientation.x,
                                       pose_geo.pose.pose.orientation.y,
                                       pose_geo.pose.pose.orientation.z));
  pose_eigen.pretranslate(Eigen::Vector3d(pose_geo.pose.pose.position.x,
                                          pose_geo.pose.pose.position.y,
                                          pose_geo.pose.pose.position.z));
  return pose_eigen;
}

extern Pose6D GeometryToPose6D(const nav_msgs::Odometry pose_geo) {
  Eigen::Quaterniond quat(pose_geo.pose.pose.orientation.w,
                          pose_geo.pose.pose.orientation.x,
                          pose_geo.pose.pose.orientation.y,
                          pose_geo.pose.pose.orientation.z);

  Pose6D p;
  p.x = pose_geo.pose.pose.position.x;
  p.y = pose_geo.pose.pose.position.y;
  p.z = pose_geo.pose.pose.position.z;
  tf::Matrix3x3(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w())).getRPY(p.roll, p.pitch, p.yaw);
  return p;
}

extern Pose6D Odom2Pose6D(const nav_msgs::Odometry::ConstPtr _odom) {
  auto tx = _odom->pose.pose.position.x;
  auto ty = _odom->pose.pose.position.y;
  auto tz = _odom->pose.pose.position.z;

  double roll, pitch, yaw;
  geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

  return Pose6D{tx, ty, tz, roll, pitch, yaw};
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


// 定义常用颜色
enum Color : int {
  BLACK,
  GRAY,
  LIGHT_RED,
  LIGHT_GREEN,
  LIGHT_BLUE,
  LIGHT_YELLOW,
  LIGHT_CYAN,
  LIGHT_MAGENTA,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE
};

const double COLOR_VALUE_MIN = 0.0;
const double COLOR_VALUE_MAX = 1.0;
const double COLOR_VALUE_MEDIAN = 0.5;
const double COLOR_VALUE_LIGHT_LOW = 0.56;
const double COLOR_VALUE_LIGHT_HIGH = 0.93;

std_msgs::ColorRGBA createColorRGBA(Color color) {
  std_msgs::ColorRGBA color_rgba;
  color_rgba.r = COLOR_VALUE_MIN;
  color_rgba.g = COLOR_VALUE_MIN;
  color_rgba.b = COLOR_VALUE_MIN;
  color_rgba.a = COLOR_VALUE_MAX;

  switch (color) {
    case BLACK:break;
    case GRAY:color_rgba.r = COLOR_VALUE_MEDIAN;
      color_rgba.g = COLOR_VALUE_MEDIAN;
      color_rgba.b = COLOR_VALUE_MEDIAN;
      break;
    case LIGHT_RED:color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
      color_rgba.g = COLOR_VALUE_LIGHT_LOW;
      color_rgba.b = COLOR_VALUE_LIGHT_LOW;
      break;
    case LIGHT_GREEN:color_rgba.r = COLOR_VALUE_LIGHT_LOW;
      color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
      color_rgba.b = COLOR_VALUE_LIGHT_LOW;
      break;
    case LIGHT_BLUE:color_rgba.r = COLOR_VALUE_LIGHT_LOW;
      color_rgba.g = COLOR_VALUE_LIGHT_LOW;
      color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
      break;
    case LIGHT_YELLOW:color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
      color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
      color_rgba.b = COLOR_VALUE_LIGHT_LOW;
      break;
    case LIGHT_CYAN:color_rgba.r = COLOR_VALUE_LIGHT_LOW;
      color_rgba.g = COLOR_VALUE_LIGHT_HIGH;
      color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
      break;
    case LIGHT_MAGENTA:color_rgba.r = COLOR_VALUE_LIGHT_HIGH;
      color_rgba.g = COLOR_VALUE_LIGHT_LOW;
      color_rgba.b = COLOR_VALUE_LIGHT_HIGH;
      break;
    case RED:color_rgba.r = COLOR_VALUE_MAX;
      break;
    case GREEN:color_rgba.g = COLOR_VALUE_MAX;
      break;
    case BLUE:color_rgba.b = COLOR_VALUE_MAX;
      break;
    case YELLOW:color_rgba.r = COLOR_VALUE_MAX;
      color_rgba.g = COLOR_VALUE_MAX;
      break;
    case CYAN:color_rgba.g = COLOR_VALUE_MAX;
      color_rgba.b = COLOR_VALUE_MAX;
      break;
    case MAGENTA:color_rgba.r = COLOR_VALUE_MAX;
      color_rgba.b = COLOR_VALUE_MAX;
      break;
    case WHITE:color_rgba.r = COLOR_VALUE_MAX;
      color_rgba.g = COLOR_VALUE_MAX;
      color_rgba.b = COLOR_VALUE_MAX;
      break;
    default:color_rgba.a = COLOR_VALUE_MIN; // hide color from view
      break;
  }

  return color_rgba;
}

#endif //SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_COMMON_H_

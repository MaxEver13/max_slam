/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-13 14:49:47
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-14 17:30:30
 */

#ifndef SRC_MAX_MAPPING_SRC_TRANSFORM_FUSION_H_
#define SRC_MAX_MAPPING_SRC_TRANSFORM_FUSION_H_

#include "utils/common.h"


class TransformFusion {
public:
    TransformFusion(ros::NodeHandle& nh);

    void lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

    void imuOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);    

private:
    // 订阅的lidar odometry话题
    std::string lidar_odom_topic_sub_;
    // 订阅的imu odometry话题
    std::string imu_odom_topic_sub_;
    // 发布的imu odometry话题
    std::string imu_odom_topic_pub_;
    // 发布的imu path话题
    std::string imu_path_topic_pub_;

    // lidar坐标系
    std::string lidar_frame_;
    // baselink坐标系
    std::string baselink_frame_;
    // map坐标系
    std::string map_frame_;
    // odometry坐标系
    std::string odometry_frame_;

    // ros句柄
    ros::NodeHandle nh_;

    // 订阅imu preintegration发布的imu odometry
    ros::Subscriber imu_odom_sub_;
    // 订阅lidar mapping发布的lidar odometry
    ros::Subscriber lidar_odom_sub_;

    // 发布imu odometry和lidar odometry融合之后的odometry和path
    ros::Publisher imu_odom_pub_;
    ros::Publisher imu_path_pub_;

    // 最新lidar里程计
    Eigen::Affine3f lidar_odom_affine_;

    // TF查询lidar坐标系与baselink坐标系的转换
    tf::TransformListener tf_listener_;
    tf::StampedTransform lidar_to_baselink_;

    // lidar odometry时间戳，初始值为-1
    double lidar_odom_time_ = -1;
    // 数据锁
    std::mutex mtx_;
    // imu odometry队列
    std::deque<nav_msgs::Odometry> imu_odom_queue_;

    // ros odometry格式Eigen::Affine
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom);
};

#endif // SRC_MAX_MAPPING_SRC_TRANSFORM_FUSION_H_

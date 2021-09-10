/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-06 14:57:02
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-10 17:52:30
 */

#ifndef SRC_MAX_MAPPING_SRC_GROUND_SEGMENTATION_NODE_H_
#define SRC_MAX_MAPPING_SRC_GROUND_SEGMENTATION_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <thread>
#include <mutex>
#include <queue>

#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>


#include <sensor_msgs/PointCloud2.h>

#include "ground_segmentation/ground_segmentation.h"


class SegmentationNode {

private:
  // 
  std::string lidar_frame_id_;

  // ros句柄
  ros::NodeHandle nh_;
  // 发布地面点云
  ros::Publisher ground_pub_;
  // 发布非地面点云
  ros::Publisher obstacle_pub_;
  // 地面分割参数
  GroundSegmentationParams params_;

  // 点云数据
  std::mutex cloud_mutex_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;



public:
  SegmentationNode(ros::NodeHandle& nh,
                   const std::string& ground_topic,
                   const std::string& obstacle_topic,
                   const GroundSegmentationParams& params,
                   const bool& latch = false) : nh_(nh), params_(params) {

    nh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "velo_link");
    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 1, latch);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_topic, 1, latch);
  }


/**
 * @function: 主函数入口，对点云进行地面分割，并发布地面点云和非地面点云消息
 * @param {*}
 * @return {*}
 */
  void run(); 


/**
 * @function: 点云消息回调
 * @param {laserCloudMsg}
 * @return {*}
 */  
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  
};

#endif //SRC_MAX_MAPPING_SRC_GROUND_SEGMENTATION_NODE_H_

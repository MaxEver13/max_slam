/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-06 14:57:02
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-21 13:05:32
 */

#ifndef SRC_MAX_MAPPING_SRC_GROUND_SEGMENTATION_H_
#define SRC_MAX_MAPPING_SRC_GROUND_SEGMENTATION_H_

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

#include "max_slam/cloud_info.h"

#include "utils/common.h"

#include "ground_segmentation/ground_segmentation.h"


class Segmentation : public ParamServer 
{
private:
  std::string cloud_info_sub_topic_;
  std::string ground_pub_topic_;
  std::string obstacle_pub_topic_;
  std::string cloud_info_pub_topic_;

  ros::Subscriber cloud_sub_;

  // 发布地面点云
  ros::Publisher ground_pub_;
  // 发布非地面点云
  ros::Publisher obstacle_pub_;
  // 分割后的非地面cloud info
  ros::Publisher cloud_info_pub_;

  // 地面分割参数
  GroundSegmentationParams params_;
  bool latch_;

  // 点云数据
  std::mutex cloud_mutex_;
  std::deque<max_slam::cloud_info> cloud_queue_;
  max_slam::cloud_info cloud_info_;

public:
  Segmentation(ros::NodeHandle& nh);
  ~Segmentation();

/**
 * @brief: 主函数入口，对点云进行地面分割，并发布地面点云和非地面点云消息
 * @param {*}
 * @return {*}
 */
  void run(); 


private:
/**
 * @brief: 点云消息回调
 * @param {max_slam::cloud_infoConstPtr} laserCloudMsg
 * @return {*}
 */  
  void cloudCallback(const max_slam::cloud_infoConstPtr& laserCloudMsg);
  
};

#endif //SRC_MAX_MAPPING_SRC_SEGMENTATION_H_

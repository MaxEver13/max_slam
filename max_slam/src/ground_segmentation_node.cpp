/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-07 15:59:19
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-10 18:21:04
 */
#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>

#include "ground_segmentation/ground_segmentation.h"
#include "max_slam/ground_segmention_node.h"

void SegmentationNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  cloud_mutex_.lock();
  cloud_queue_.push(laserCloudMsg);
  cloud_mutex_.unlock();
}

void SegmentationNode::run() {
  while (1)
  {
    while (!cloud_queue_.empty())
    {
      // ros消息转换成pcl点云
      cloud_mutex_.lock();
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*cloud_queue_.front(), *cloud_ptr);
      cloud_queue_.pop();
      cloud_mutex_.unlock();

      if (cloud_ptr->empty()) {
        ROS_ERROR("SegmentationNode: cloud is empty !!!");
        continue;
      }

      // 地面分割
      GroundSegmentation segmenter(params_);
      std::vector<int> labels;

      segmenter.segment(*cloud_ptr, &labels);
      pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
      ground_cloud.header = cloud_ptr->header;
      ground_cloud.header.frame_id = lidar_frame_id_;
      obstacle_cloud.header = cloud_ptr->header;
      obstacle_cloud.header.frame_id = lidar_frame_id_;
      for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        if (labels[i] == 1) ground_cloud.push_back((*cloud_ptr)[i]);
        else obstacle_cloud.push_back((*cloud_ptr)[i]);
      }

      // 发布点云消息
      ground_pub_.publish(ground_cloud);
      obstacle_pub_.publish(obstacle_cloud);
    }

    // 主线程sleep
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);    
  }  
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");

  ros::NodeHandle nh("~");

  // Do parameter stuff.
  GroundSegmentationParams params;
  nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  // ROS_INFO("n_bins: %d", params.n_bins);
  
  // Params that need to be squared.
  double r_min, r_max, max_fit_error;
  if (nh.getParam("r_min", r_min)) {
    params.r_min_square = r_min*r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max*r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic;
  bool latch;
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param<bool>("latch", latch, false);

  // SegmentationNode
  SegmentationNode node(nh, ground_topic, obstacle_topic, params, latch);

  // 开启点云消息回调
  ros::Subscriber cloud_sub;
  cloud_sub = nh.subscribe(input_topic, 1, &SegmentationNode::cloudCallback, &node);

  // 开启地面分割线程
  std::thread seg_thread(&SegmentationNode::run, &node);

  // 保证回调生效
  ros::spin();
  return 0;
}

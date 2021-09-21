/*
 * @Descripttion: 1.接收原始点云消息 2.执行地面点云分割并发布地面点云和非地面点云消息
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-07 15:59:19
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-21 13:08:42
 */

// #include <pcl/io/ply_io.h>
#include "max_slam/ground_seg.h"


Segmentation::Segmentation(ros::NodeHandle& nh) 
{
    // 由于子类不能在初始化列表给基类成员初始化
    // 这里采用赋值
    nh_ = nh;

    nh_.param<bool>("visualize", params_.visualize, false);
    nh_.param<int>("n_bins", params_.n_bins, 120);
    nh_.param<int>("n_segments", params_.n_segments, 360);
    nh_.param<double>("max_dist_to_line", params_.max_dist_to_line, 0.05);
    nh_.param<double>("max_slope", params_.max_slope, 0.3);
    nh_.param<double>("long_threshold", params_.long_threshold, 1.0);
    nh_.param<double>("max_long_height", params_.max_long_height, 0.1);
    nh_.param<double>("max_start_height", params_.max_start_height, 0.2);
    nh_.param<double>("sensor_height", params_.sensor_height, 1.8);
    nh_.param<double>("line_search_angle", params_.line_search_angle, 0.1);
    nh_.param<int>("n_threads", params_.n_threads, 4);

    ROS_INFO("n_bins: %d", params_.n_bins);
    
    // Params that need to be squared.
    double r_min, r_max, max_fit_error;
    if (nh_.getParam("r_min", r_min)) {
      params_.r_min_square = r_min*r_min;
    }
    if (nh_.getParam("r_max", r_max)) {
      params_.r_max_square = r_max*r_max;
    }
    if (nh_.getParam("max_fit_error", max_fit_error)) {
      params_.max_error_square = max_fit_error * max_fit_error;
    }

    nh_.param<std::string>("cloud_info_sub_topic", cloud_info_sub_topic_, "/input_cloud");
    nh_.param<std::string>("ground_pub_topic", ground_pub_topic_, "/ground_cloud");
    nh_.param<std::string>("obstacle_pub_topic", obstacle_pub_topic_, "/obstacle_cloud");
    nh_.param<std::string>("cloud_info_pub_topic", cloud_info_pub_topic_, "/ground_seg/cloud_info");
    
    nh_.param<bool>("latch", latch_, false);

    // 开启点云消息回调    
    cloud_sub_ = nh_.subscribe<max_slam::cloud_info>(cloud_info_sub_topic_, 1, &Segmentation::cloudCallback, this, ros::TransportHints().tcpNoDelay());

    ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_pub_topic_, 1, latch_);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_pub_topic_, 1, latch_);   
    cloud_info_pub_ = nh_.advertise<max_slam::cloud_info>(cloud_info_pub_topic_, 1);
}

Segmentation::~Segmentation()
{
  
}

void Segmentation::cloudCallback(const max_slam::cloud_infoConstPtr& laserCloudMsg) {
  // ROS_INFO("cloud info Time: %f", ROS_TIME(laserCloudMsg));
  cloud_mutex_.lock();
  cloud_queue_.push_back(*laserCloudMsg);
  cloud_mutex_.unlock();
}

void Segmentation::run() {
  while (1)
  {
    while (!cloud_queue_.empty())
    {
      // ros消息转换成pcl点云
      cloud_mutex_.lock();
      max_slam::cloud_info currCloudInfo = std::move(cloud_queue_.front());
      cloud_queue_.pop_front();
      cloud_mutex_.unlock();      

      pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
      pcl::fromROSMsg(currCloudInfo.cloud_deskewed, *cloud_ptr);  

      if (cloud_ptr->empty()) {
        ROS_ERROR("SegmentationNode: cloud is empty !!!");
        continue;
      }

      // 地面分割
      GroundSegmentation segmenter(params_);
      std::vector<int> labels;

      segmenter.segment(*cloud_ptr, &labels);

      pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr obstacle_cloud(new pcl::PointCloud<PointType>());

      for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        if (labels[i] == 1) ground_cloud->push_back((*cloud_ptr)[i]);
        else obstacle_cloud->push_back((*cloud_ptr)[i]);
      }

      // 发布点云消息
      cloud_info_.cloud_obstacle = publishCloud(&obstacle_pub_, obstacle_cloud, currCloudInfo.header.stamp, lidar_frame_);
      cloud_info_.cloud_ground = publishCloud(&ground_pub_, ground_cloud, currCloudInfo.header.stamp, lidar_frame_);
      cloud_info_pub_.publish(cloud_info_);
    }

    // 主线程sleep
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);    
  }  
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");

  ROS_INFO("\033[1;32m----> Ground Segmentation Started.\033[0m");

  // 局部空间解析参数
  ros::NodeHandle nh("~");

  // Segmentation Node
  Segmentation seg(nh);  

  // 开启地面分割线程
  std::thread seg_thread(&Segmentation::run, &seg);

  // 保证回调生效
  ros::spin();
  
  return 0;
}

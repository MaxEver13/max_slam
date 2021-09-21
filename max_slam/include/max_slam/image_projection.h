/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-16 14:55:54
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-19 22:56:45
 */

#ifndef SRC_MAX_MAPPING_SRC_IMAGE_PROJECTION_H_
#define SRC_MAX_MAPPING_SRC_IMAGE_PROJECTION_H_

#include "utils/common.h"
#include "max_slam/cloud_info.h"


#define QUEUE_SIZE 2000

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

class ImageProjection : public ParamServer
{
private:    
    std::mutex imu_mtx_;    // imu数据锁
    std::mutex odom_mtx_; // odometry数据锁

    // 订阅主题
    std::string odom_topic_sub_;
    std::string cloud_topic_sub_;

    // 发布主题
    std::string cloud_topic_pub_;
    std::string cloud_info_topic_pub_;

    // 订阅9轴imu数据
    ros::Subscriber imu_sub_;
    std::deque<sensor_msgs::Imu> imu_queue_;

    // 订阅里程计数据(由imu pre-integration 节点发布的odometry 已经转换到lidar坐标系)
    ros::Subscriber odom_sub_;
    std::deque<nav_msgs::Odometry> odom_queue_;

    // 订阅点云数据(由地面分割节点发布的非地面点云)
    ros::Subscriber cloud_sub_;
    std::deque<sensor_msgs::PointCloud2> cloud_queue_;
    sensor_msgs::PointCloud2 cur_cloud_msg_;

    // 发布去畸变点云用于显示
    ros::Publisher deskewed_cloud_pub_;
    // 发布去畸变点云的cloud info: 含imu姿态信息，点云位姿初始估计等
    ros::Publisher cloud_info_pub_;

    // 原始点云
    pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud_in_;
    // 去畸变后的点云
    pcl::PointCloud<PointType>::Ptr   deskewed_cloud_;

    // 当前scan扫描的起止时刻
    max_slam::cloud_info cloud_info_;
    std_msgs::Header cloud_header_;
    double time_scan_cur_;
    double time_scan_end_;
    
    // 检查时间戳，是否可以做去运动畸变，1表示支持时间戳
    int deskew_flag_;

    // 当前scan扫描起止时刻的旋转量,用于旋转去畸变
    int imu_pointer_cur_; 
    double *imu_time_ = new double[QUEUE_SIZE];
    double *imu_rot_x_ = new double[QUEUE_SIZE];
    double *imu_rot_y_ = new double[QUEUE_SIZE];
    double *imu_rot_z_ = new double[QUEUE_SIZE];

    // 当前scan扫描起止时刻的位姿增量，用于平移去畸变
    bool odom_deskew_flag_;
    float odom_incre_x_;
    float odom_incre_y_;
    float odom_incre_z_;

    // 当前scan中的第一个点   
    bool first_point_flag_;
    Eigen::Affine3f trans_start_inverse_;


public:
    ImageProjection(ros::NodeHandle& nh);
    ~ImageProjection();

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);

private:
    void allocateMemory();
    void resetParameters();

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

    void imuDeskewInfo();

    void odomDeskewInfo();   

    bool deskewInfo();

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);

    PointType deskewPoint(PointType *point, double relTime); 

    void projectPointCloud();

    void publishClouds();

};




#endif // SRC_MAX_MAPPING_SRC_IMAGE_PROJECTION_H_
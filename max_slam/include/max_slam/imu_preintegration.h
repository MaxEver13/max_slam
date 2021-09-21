/*
 * @Descripttion: imu 预积分
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-13 14:50:16
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-19 22:57:30
 */

#ifndef SRC_MAX_MAPPING_SRC_IMU_PREINTEGRATION_H_
#define SRC_MAX_MAPPING_SRC_IMU_PREINTEGRATION_H_

#include "utils/common.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class IMUPreintegration : public ParamServer 
{
private:
    
    std::mutex mtx_;

    // ros topic
    std::string lidar_odom_topic_sub_;
    std::string imu_odom_topic_pub_;

    // 订阅imu数据
    ros::Subscriber imu_sub_;
    // 订阅lidar odometry
    ros::Subscriber odometry_sub_;
    // 发布imu预积分odom
    ros::Publisher imu_odometry_pub_;

    // 初始化标志位
    bool system_initialized_ = false;

    // 先验噪声协方差
    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise2_;
    gtsam::Vector noise_model_between_bias_;

    // imu预积分用于因子图优化位姿，速度，bias
    gtsam::PreintegratedImuMeasurements *imu_integrator_opt_;
    // imu预积分用于预测位姿
    gtsam::PreintegratedImuMeasurements *imu_integrator_imu_;

    // imu数据队列用于因子图优化位姿，速度，bias
    std::deque<sensor_msgs::Imu> imu_que_opt_;
    // imu数据队列用于预测位姿
    std::deque<sensor_msgs::Imu> imu_que_imu_;

    // 因子图优化过程中的状态变量
    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::imuBias::ConstantBias prev_bias_;
    // 状态包含旋转，平移，速度
    gtsam::NavState prev_state_;
    
    // 因子图优化之后用于计算odom的
    gtsam::NavState prev_state_odom_;
    gtsam::imuBias::ConstantBias prev_bias_odom_;

    // ISAM2优化求解器，因子图，顶点值
    gtsam::ISAM2 optimizer_;
    gtsam::NonlinearFactorGraph graph_factors_;
    gtsam::Values graph_values_;

    // 第一次优化完成标志位
    bool first_opt_done_ = false;

    const double delta_t = 0;
    int key = 1;
    // 上一帧imu数据的时间戳
    double lastImuT_imu_ = -1;
    double lastImuT_opt_ = -1;

    // 外参默认值为imu坐标系与lidar坐标系重合
    // imu到lidar的转换T_velo_imu
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
    // lidar到imu的转换T_imu_velo
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
    

public:
    IMUPreintegration(ros::NodeHandle& nh);

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_raw);

private:
    void resetOptimization();

    void resetParams();

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
 
};

#endif // SRC_MAX_MAPPING_SRC_IMU_PREINTEGRATION_H_

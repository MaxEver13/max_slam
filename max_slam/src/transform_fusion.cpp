/*
 * @Descripttion: 将imu预积分的odom与lidar odom融合,得到最新的lidar odom预测位姿
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-13 15:52:58
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-19 23:05:01
 */

#include "max_slam/transform_fusion.h"

TransformFusion::TransformFusion(ros::NodeHandle& nh)
{
    // 由于子类不能在初始化列表给基类成员初始化
    // 这里采用赋值
    nh_ = nh;

    //　ros topic
    nh_.param<std::string>("lidar_odom_topic_sub", lidar_odom_topic_sub_, "/lidar_mapping/odometry");
    nh_.param<std::string>("imu_odom_topic_sub", imu_odom_topic_sub_, "/odometry/imu_incremental");
    nh_.param<std::string>("imu_odom_topic_pub", imu_odom_topic_pub_, "/odometry/imu");
    nh_.param<std::string>("imu_path_topic_pub", imu_path_topic_pub_, "/imu_path");

    ROS_INFO("baselinkFrame: %s", baselink_frame_.c_str());

    // 如果雷达坐标系跟baselink坐标系不一致
    if(lidar_frame_ != baselink_frame_)
    {
        try
        {
            // 查询到坐标转换lidar_to_baselink_
            tf_listener_.waitForTransform(lidar_frame_, baselink_frame_, ros::Time(0), ros::Duration(3.0));
            tf_listener_.lookupTransform(lidar_frame_, baselink_frame_, ros::Time(0), lidar_to_baselink_);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }

    lidar_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(lidar_odom_topic_sub_, 5, &TransformFusion::lidarOdometryCallback, this, ros::TransportHints().tcpNoDelay());
    imu_odom_sub_   = nh_.subscribe<nav_msgs::Odometry>(imu_odom_topic_sub_, 2000, &TransformFusion::imuOdometryCallback,   this, ros::TransportHints().tcpNoDelay());

    // 这里发布的其实是lidar odometry
    imu_odom_pub_   = nh_.advertise<nav_msgs::Odometry>(imu_odom_topic_pub_, 2000);
    imu_path_pub_   = nh_.advertise<nav_msgs::Path>(imu_path_topic_pub_, 1);
}

TransformFusion::~TransformFusion()
{
    
}

/**
 * @brief: odometry转换成Eigen::Affine3f
 * @param {Odometry} odom
 * @return {*}
 */
Eigen::Affine3f TransformFusion::odom2affine(nav_msgs::Odometry odom)
{
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}

/**
 * @brief: lidar odometry消息回调函数
 * @param {nav_msgs::Odometry::ConstPtr}　odomMsg
 * @return {*}
 */
void TransformFusion::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // 获取当前lidar odometry消息
    std::lock_guard<std::mutex> lock(mtx_);
    // 最新lidar里程计
    lidar_odom_affine_ = odom2affine(*odomMsg);
    // 更新lidar odometry时间
    lidar_odom_time_ = odomMsg->header.stamp.toSec();
}

/**
 * @brief: 将imu预积分的odom与lidar odom融合,得到最新的lidar odom预测位姿
 * @param {*}　imu_odom_queue_: 注意这里保存的odom都是imu转换到lidar坐标系下的
 * @return {*}
 */
void TransformFusion::imuOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // static tf
    // 将map坐标系与odom坐标系设为同一个
    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, map_frame_, odometry_frame_));

    // 队列加锁保护
    std::lock_guard<std::mutex> lock(mtx_);

    imu_odom_queue_.push_back(*odomMsg);

    // 如果没有收到lidar odometry直接返回
    if (lidar_odom_time_ == -1)
        return;

    // 丢掉早于当前lidar odom时间的imu odom数据，对齐到最新lidar odometry时刻
    // 默认imu odom数据频率很高
    while (!imu_odom_queue_.empty())
    {
        if (imu_odom_queue_.front().header.stamp.toSec() <= lidar_odom_time_)
            imu_odom_queue_.pop_front();
        else
            break;
    }
    // 计算imu odom位姿变换的增量
    Eigen::Affine3f imuOdomAffineFront = odom2affine(imu_odom_queue_.front());
    Eigen::Affine3f imuOdomAffineBack = odom2affine(imu_odom_queue_.back());
    Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
    // 通过最新的lidar odom乘上imu odom的增量重新计算最新的imu位姿
    Eigen::Affine3f imuOdomAffineLast = lidar_odom_affine_ * imuOdomAffineIncre;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
    
    // 发布最新的odometry
    nav_msgs::Odometry lidarOdometry = imu_odom_queue_.back();
    lidarOdometry.pose.pose.position.x = x;
    lidarOdometry.pose.pose.position.y = y;
    lidarOdometry.pose.pose.position.z = z;
    lidarOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    imu_odom_pub_.publish(lidarOdometry);

    // 发布tf odom 到　baselink
    static tf::TransformBroadcaster tfOdom2BaseLink;
    tf::Transform tCur;
    tf::poseMsgToTF(lidarOdometry.pose.pose, tCur);
    if(lidar_frame_ != baselink_frame_)
        tCur = tCur * lidar_to_baselink_;
    tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometry_frame_, baselink_frame_);
    tfOdom2BaseLink.sendTransform(odom_2_baselink);

    // 发布最近一段path
    static nav_msgs::Path imuPath;
    static double last_path_time = -1;
    double imuTime = imu_odom_queue_.back().header.stamp.toSec();
    if (imuTime - last_path_time > 0.1)
    {
        last_path_time = imuTime;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = imu_odom_queue_.back().header.stamp;
        pose_stamped.header.frame_id = odometry_frame_;
        pose_stamped.pose = lidarOdometry.pose.pose;
        imuPath.poses.push_back(pose_stamped);
        // 丢弃早于当前lidar odom时刻前一秒的path
        while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidar_odom_time_ - 1.0)
            imuPath.poses.erase(imuPath.poses.begin());
        if (imu_path_pub_.getNumSubscribers() != 0)
        {
            imuPath.header.stamp = imu_odom_queue_.back().header.stamp;
            imuPath.header.frame_id = odometry_frame_;
            imu_path_pub_.publish(imuPath);
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform fusion");

    ROS_INFO("\033[1;32m----> Transform Fusion Started.\033[0m");

    // 局部空间解析参数
    ros::NodeHandle nh("~");

    TransformFusion TF(nh);

    ros::spin();

    return 0;
}
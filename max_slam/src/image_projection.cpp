/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-16 16:10:04
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-21 13:13:37
 */

#include "max_slam/image_projection.h"

ImageProjection::ImageProjection(ros::NodeHandle& nh) : deskew_flag_(0) 
{
    // 由于子类不能在初始化列表给基类成员初始化
    // 这里采用赋值
    nh_ = nh;

    // 订阅的主题
    nh_.param<std::string>("odom_topic_sub", odom_topic_sub_, "/odometry/imu_incremental");
    nh_.param<std::string>("cloud_topic_sub", cloud_topic_sub_, "/obstacle_cloud");
    // 发布的主题
    nh_.param<std::string>("cloud_topic_pub", cloud_topic_pub_, "/deskew/cloud_deskewed");
    nh_.param<std::string>("cloud_info_topic_pub", cloud_info_topic_pub_, "/deskew/cloud_info");

    ROS_INFO("lidarFrame: %s", lidar_frame_.c_str());    

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 2000, &ImageProjection::imuCallback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_sub_, 2000, &ImageProjection::odometryCallback, this, ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 5, &ImageProjection::laserCloudCallback, this, ros::TransportHints().tcpNoDelay());

    deskewed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_topic_pub_, 1);
    cloud_info_pub_ = nh_.advertise<max_slam::cloud_info> (cloud_info_topic_pub_, 1);

    allocateMemory();

    resetParameters();

}

ImageProjection::~ImageProjection()
{

}


/**
 * @function: 变量初始化
 * @param {*}
 * @return {*}
 */
void ImageProjection::allocateMemory()
{
    // 初始化指针
    laser_cloud_in_.reset(new pcl::PointCloud<PointXYZIRT>());
    deskewed_cloud_.reset(new pcl::PointCloud<PointType>());
    // 去畸变点云数据大小
    deskewed_cloud_->points.resize(N_SCAN*HORIZON_SCAN);  
}

/**
 * @brief: 每一帧点云处理完之后需要将相关参数重置一下
 * @param {*}
 * @return {*}
 */
void ImageProjection::resetParameters()
{
    laser_cloud_in_->clear();
    // 注意clear之后，需要把点云的size初始化
    deskewed_cloud_->clear();
    deskewed_cloud_->points.resize(N_SCAN*HORIZON_SCAN);

    // scan去畸变标志位这里不用每次重置，如果第一次判断不带时间戳，后面都不支持去畸变了
    // deskew_flag_ = 0;    
    first_point_flag_ = true;
    odom_deskew_flag_ = false;

    // scan扫描起止期间imu数据清零
    imu_pointer_cur_ = 0;
    for (int i = 0; i < QUEUE_SIZE; ++i)
    {
        imu_time_[i] = 0;
        imu_rot_x_[i] = 0;
        imu_rot_y_[i] = 0;
        imu_rot_z_[i] = 0;
    }
}



/**
 * @brief: 订阅imu原始数据，转换到lidar坐标系，并保存到队列
 * @param {*}
 * @return {*}
 */
void ImageProjection::imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg) 
{
    // 将imu数据转到雷达坐标系
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    std::lock_guard<std::mutex> lock(imu_mtx_);
    imu_queue_.push_back(thisImu);

    // debug IMU data
    // cout << std::setprecision(6);
    // cout << "IMU acc: " << endl;
    // cout << "x: " << thisImu.linear_acceleration.x << 
    //       ", y: " << thisImu.linear_acceleration.y << 
    //       ", z: " << thisImu.linear_acceleration.z << endl;
    // cout << "IMU gyro: " << endl;
    // cout << "x: " << thisImu.angular_velocity.x << 
    //       ", y: " << thisImu.angular_velocity.y << 
    //       ", z: " << thisImu.angular_velocity.z << endl;
    // double imuRoll, imuPitch, imuYaw;
    // tf::Quaternion orientation;
    // tf::quaternionMsgToTF(thisImu.orientation, orientation);
    // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    // cout << "IMU roll pitch yaw: " << endl;
    // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
}

/**
 * @brief: 订阅来自imu pre-integration的里程计,已经转到了lidar坐标系,并保存到队列
 * @param {*}
 * @return {*}
 */
void ImageProjection::odometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) 
{
    std::lock_guard<std::mutex> lock(odom_mtx_);
    odom_queue_.push_back(*odomMsg);
}


/**
 * @brief: 订阅来自地面分割节点的点云数据，并处理
 * @param {*}
 * @return {*}
 */
void ImageProjection::laserCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) 
{
    // 保存点云数据到队列，获取队列中第一帧点云并检查消息ring通道和时间戳
    if (!cachePointCloud(cloudMsg))
        return;

    // 准备给点云去运动畸变的数据
    if (!deskewInfo())
        return;

    // 点云去畸变
    projectPointCloud();

    // 发布去畸变后的点云以及cloud info
    publishClouds();

    // 每次处理完一帧点云都要重置参数
    resetParameters();
}


bool ImageProjection::cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    // ROS_INFO("PointCloud Time: %f", ROS_TIME(laserCloudMsg));
    // 保存点云数据到队列，如果队列中数据小于2帧，不做处理
    cloud_queue_.push_back(*laserCloudMsg);
    if (cloud_queue_.size() <= 2)
        return false;
    
    // 将队列中第一帧取出，转换成pcl点云格式
    cur_cloud_msg_ = std::move(cloud_queue_.front());
    cloud_queue_.pop_front();
    pcl::moveFromROSMsg(cur_cloud_msg_, *laser_cloud_in_);   

    // 获取当前scan扫描点的起始时间和结束时间
    cloud_header_ = cur_cloud_msg_.header;
    time_scan_cur_ = cloud_header_.stamp.toSec();
    time_scan_end_ = time_scan_cur_ + laser_cloud_in_->points.back().time;

    // check dense flag
    if (laser_cloud_in_->is_dense == false)
    {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    // 检查ring通道
    static int ringFlag = 0;
    if (ringFlag == 0)
    {
        ringFlag = -1;
        for (int i = 0; i < (int)cur_cloud_msg_.fields.size(); ++i)
        {
            if (cur_cloud_msg_.fields[i].name == "ring")
            {
                ringFlag = 1;
                break;
            }
        }
        if (ringFlag == -1)
        {
            ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
            ros::shutdown();
        }
    }
    
 
    // 检查点云时间戳，判断是否可以做去运动畸变
    if (deskew_flag_ == 0)
    {
        deskew_flag_ = -1;
        for (auto &field : cur_cloud_msg_.fields)
        {
            if (field.name == "time" || field.name == "t")
            {
                deskew_flag_ = 1;
                break;
            }
        }
        if (deskew_flag_ == -1)
            ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return true;
}


/**
 * @brief: 利用当前scan扫描时间段[time_scan_cur - 10ms，　time_scan_end + 10ms]内的imu角速度速度计算
 *         该时间段内的roll,pitch,yaw,用于计算scan内的每个点到起始点的旋转变换
 * @param {*}
 * @return {*}
 */
void ImageProjection::imuDeskewInfo()
{
    // 点云的imu姿态数据不可用
    cloud_info_.imuAvailable = false;

    // 丢弃掉早于当前scan起始时刻10ms的imu数据
    while (!imu_queue_.empty())
    {
        if (imu_queue_.front().header.stamp.toSec() < time_scan_cur_ - 0.01)
            imu_queue_.pop_front();
        else
            break;
    }

    if (imu_queue_.empty())
        return;

    // 到这里已经确保了
    imu_pointer_cur_ = 0;

    for (int i = 0; i < (int)imu_queue_.size(); ++i)
    {
        sensor_msgs::Imu thisImuMsg = imu_queue_[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();

        // 得到这帧scan起始时刻的roll,pitch,yaw
        if (currentImuTime <= time_scan_cur_)
            imuRPY2rosRPY(&thisImuMsg, &cloud_info_.imuRollInit, &cloud_info_.imuPitchInit, &cloud_info_.imuYawInit);

        // 当前imu时间戳超过这帧scan的结束时刻10ms,跳出循环
        if (currentImuTime > time_scan_end_ + 0.01)
            break;

        // 第一次保存，第一帧的旋转角度为0
        if (imu_pointer_cur_ == 0){
            imu_rot_x_[0] = 0;
            imu_rot_y_[0] = 0;
            imu_rot_z_[0] = 0;
            imu_time_[0] = currentImuTime;
            ++imu_pointer_cur_;
            continue;
        }

        // 获得角速度
        double angular_x, angular_y, angular_z;
        imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        // 对角速度一直积分，一直积分到time_scan_end_加10ms
        double timeDiff = currentImuTime - imu_time_[imu_pointer_cur_-1];
        imu_rot_x_[imu_pointer_cur_] = imu_rot_x_[imu_pointer_cur_-1] + angular_x * timeDiff;
        imu_rot_y_[imu_pointer_cur_] = imu_rot_y_[imu_pointer_cur_-1] + angular_y * timeDiff;
        imu_rot_z_[imu_pointer_cur_] = imu_rot_z_[imu_pointer_cur_-1] + angular_z * timeDiff;
        imu_time_[imu_pointer_cur_] = currentImuTime;
        ++imu_pointer_cur_;
    }
    // for循环最后加了一次，这里要减掉
    --imu_pointer_cur_;

    // 如果imu_pointer_cur_小于0,说明没有imu数据可用，直接return，后面cloud_info_.imuAvailable不会置为true
    if (imu_pointer_cur_ <= 0)
        return;

    // 点云信息的imu数据可用
    cloud_info_.imuAvailable = true;
}

/**
 * @brief: 1.利用imu pre-integration odometry得到当前帧点云的初始位姿估计
 *         2.利用odom计算用于去畸变的平移信息
 * @param {*}
 * @return {*}
 */
void ImageProjection::odomDeskewInfo()
{
    // 首先认为odom不可用
    cloud_info_.odomAvailable = false;

    while (!odom_queue_.empty())
    {
        if (odom_queue_.front().header.stamp.toSec() < time_scan_cur_ - 0.01)
            odom_queue_.pop_front();
        else
            break;
    }

    // 为空直接返回
    if (odom_queue_.empty())
        return;
    // 确保可以拿到scan起始时刻的 odometry
    if (odom_queue_.front().header.stamp.toSec() > time_scan_cur_)
        return;

    // 得到scan起始时刻的imu pre-integration odometry
    nav_msgs::Odometry startOdomMsg;
    for (int i = 0; i < (int)odom_queue_.size(); ++i)
    {
        startOdomMsg = odom_queue_[i];

        if (ROS_TIME(&startOdomMsg) < time_scan_cur_)
            continue;
        else
            break;
    }

    
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // 利用imu pre-integration odometry得到当前帧scan的初始位姿估计，用于mapping中做位姿预估
    cloud_info_.initialGuessX = startOdomMsg.pose.pose.position.x;
    cloud_info_.initialGuessY = startOdomMsg.pose.pose.position.y;
    cloud_info_.initialGuessZ = startOdomMsg.pose.pose.position.z;
    cloud_info_.initialGuessRoll  = roll;
    cloud_info_.initialGuessPitch = pitch;
    cloud_info_.initialGuessYaw   = yaw;

    // 当前点云的odom预测可用
    cloud_info_.odomAvailable = true;

    // 得到当前scan扫描结束时刻的odometry

    // odom信息暂时不能用于去畸变
    odom_deskew_flag_ = false;
    // 确保可以拿到扫描结束时刻的odometry
    if (odom_queue_.back().header.stamp.toSec() < time_scan_end_)
        return;

    nav_msgs::Odometry endOdomMsg;

    for (int i = 0; i < (int)odom_queue_.size(); ++i)
    {
        endOdomMsg = odom_queue_[i];

        if (ROS_TIME(&endOdomMsg) < time_scan_end_)
            continue;
        else
            break;
    }
    // 如果当前scan扫描起止时刻的odometry的协方差不一致，直接返回，不计算odom位姿增量用于去畸变
    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        return;

    // 当前scan扫描起止时刻的位姿增量
    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    float rollIncre, pitchIncre, yawIncre;
    // 这里只利用了平移信息用于去畸变
    pcl::getTranslationAndEulerAngles(transBt, odom_incre_x_, odom_incre_y_, odom_incre_z_, rollIncre, pitchIncre, yawIncre);
    // odom平移信息可以用于去畸变
    odom_deskew_flag_ = true;
}

/**
 * @brief: 准备运动去畸变的信息
 * @param {*}
 * @return {*}
 */
bool ImageProjection::deskewInfo()
{
    std::lock_guard<std::mutex> lock1(imu_mtx_);
    std::lock_guard<std::mutex> lock2(odom_mtx_);

    // 确保在当前帧扫描起始时刻到终止时刻，imu数据可用
    if (imu_queue_.empty() || imu_queue_.front().header.stamp.toSec() > time_scan_cur_ || imu_queue_.back().header.stamp.toSec() < time_scan_end_)
    {
        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }

    // 准备用于去畸变的旋转信息
    imuDeskewInfo();

    // 准备用于去畸变的平移信息
    odomDeskewInfo();

    return true;
}


void ImageProjection::findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    // 找到
    int imuPointerFront = 0;
    while (imuPointerFront < imu_pointer_cur_)
    {
        if (pointTime < imu_time_[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    // imuPointerFront == 0: 表示当前雷达点时间戳小于整个scan扫描期间imu数据第一帧的时间
    // pointTime > imu_time_[imuPointerFront]: 表示当前雷达点时间戳大于整个scan扫描期间最后一帧imu数据时间
    // 但是之前我们已经保证了整个scan扫描期间imu数据都可用，这种情况不应该出现才对，打印log观察下
    if (pointTime > imu_time_[imuPointerFront] || imuPointerFront == 0)
    {
        ROS_ERROR("This shouldn't happen ... imuPointerFront: %d, pointTime: %f, imuTime: %f", imuPointerFront, pointTime, imu_time_[imuPointerFront]);
        *rotXCur = imu_rot_x_[imuPointerFront];
        *rotYCur = imu_rot_y_[imuPointerFront];
        *rotZCur = imu_rot_z_[imuPointerFront];
    } else {
        // imu_time_[imuPointerFront] > pointTime: imu时间比当前雷达点时间戳大
        // 找到前一帧imu数据进行线性插值
        int imuPointerBack = imuPointerFront - 1;
        // 线性插值计算：https://blog.csdn.net/liuweiyuxiang/article/details/86525249
        double ratioFront = (pointTime - imu_time_[imuPointerBack]) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);
        double ratioBack = (imu_time_[imuPointerFront] - pointTime) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);
        *rotXCur = imu_rot_x_[imuPointerFront] * ratioFront + imu_rot_x_[imuPointerBack] * ratioBack;
        *rotYCur = imu_rot_y_[imuPointerFront] * ratioFront + imu_rot_y_[imuPointerBack] * ratioBack;
        *rotZCur = imu_rot_z_[imuPointerFront] * ratioFront + imu_rot_z_[imuPointerBack] * ratioBack;
    }
}

void ImageProjection::findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
{
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    // 如果传感器运动速度较慢，比如低速场景，差不多人行走速度，平移信息用于去畸变效果不明显，可以不用考虑
    // 但是如果是快速运动还是需要考虑的，比如车载场景
    // 确保odom数据可用
    if (cloud_info_.odomAvailable == false || odom_deskew_flag_ == false)
        return;

    // 计算雷达点占整个扫描周期的比例
    float ratio = relTime / (time_scan_end_ - time_scan_cur_);
    // 假设短时间内匀速运动，计算平移量
    *posXCur = ratio * odom_incre_x_;
    *posYCur = ratio * odom_incre_y_;
    *posZCur = ratio * odom_incre_z_;
}

/**
 * @brief: 雷达点去运动畸变: 
 * @param {PointType} *point 雷达点
 * @param {double} relTime 雷达点相对该帧scan起始时刻的的时间
 * @return {*}
 */
PointType ImageProjection::deskewPoint(PointType *point, double relTime)
{
    // 时间戳不可用或者imu数据不可用，不能做去运动畸变
    if (deskew_flag_ == -1 || cloud_info_.imuAvailable == false)
        return *point;

    // 该雷达点的时间戳 = 起始时刻　＋　相对运动时间
    double pointTime = time_scan_cur_ + relTime;

    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    // 如果是当前scan中的第一个雷达点
    if (first_point_flag_ == true)
    {
        // 记录下第一个点的位姿
        trans_start_inverse_ = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
        first_point_flag_ = false;
    }

    // 计算当前雷达点到该帧scan的第一个点的位姿变换
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = trans_start_inverse_ * transFinal;

    // 将当前点转换到第一个点坐标系下，对齐到每帧scan的起始时刻
    PointType newPoint;
    newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    newPoint.intensity = point->intensity;

    return newPoint;
}


/**
 * @brief: 每帧scan点云运动去畸变
 * @param {*}
 * @return {*}
 */
void ImageProjection::projectPointCloud()
{
    // 当前帧点云的所有点
    int cloudSize = laser_cloud_in_->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        PointType thisPoint;
        thisPoint.x = laser_cloud_in_->points[i].x;
        thisPoint.y = laser_cloud_in_->points[i].y;
        thisPoint.z = laser_cloud_in_->points[i].z;
        thisPoint.intensity = laser_cloud_in_->points[i].intensity;

        // 检查lidar range        
        float range = pointDistance(thisPoint);
        if (range < LIDAR_MIN_RANGE || range > LIDAR_MAX_RANGE)
            continue;

        // 检查竖直方向线数，多少行
        int rowIdn = laser_cloud_in_->points[i].ring;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        // 检查水平方向线数，多少列
        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        static float ang_res_x = 360.0/float(HORIZON_SCAN);
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + HORIZON_SCAN/2;
        if (columnIdn >= HORIZON_SCAN)
            columnIdn -= HORIZON_SCAN;

        if (columnIdn < 0 || columnIdn >= HORIZON_SCAN)
            continue;

        // 雷达点去运动畸变
        thisPoint = deskewPoint(&thisPoint, laser_cloud_in_->points[i].time);

        // 保存去畸变后的点云
        int index = columnIdn + rowIdn * HORIZON_SCAN;
        deskewed_cloud_->points[index] = thisPoint;
    }
}


/**
 * @brief: 发布cloud_info msg用于lidar mapping 以及　去畸变的点云用于显示
 * @param {*}
 * @return {*}
 */
void ImageProjection::publishClouds()
{
    cloud_info_.header = cloud_header_;    
    cloud_info_.cloud_deskewed  = publishCloud(&deskewed_cloud_pub_, deskewed_cloud_, cloud_header_.stamp, lidar_frame_);
    cloud_info_pub_.publish(cloud_info_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image projection");

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    // 局部空间解析参数
    ros::NodeHandle nh("~");

    ImageProjection IP(nh);

    ros::spin();

    return 0;
}



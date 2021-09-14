/*
 * @Descripttion: 
 * @version: 
 * @Author: Jiawen Ji
 * @Date: 2021-09-13 20:31:07
 * @LastEditors: Jiawen Ji
 * @LastEditTime: 2021-09-14 18:12:37
 */

#include "max_slam/imu_preintegration.h"

IMUPreintegration::IMUPreintegration(ros::NodeHandle& nh) : nh_(nh) {

    nh_.param<float>("imuAccNoise", imu_acc_noise_, 0.01);
    nh_.param<float>("imuGyrNoise", imu_gyr_noise_, 0.001);
    nh_.param<float>("imuAccBiasN", imu_acc_biasN_, 0.0002);
    nh_.param<float>("imuGyrBiasN", imu_gyr_biasN_, 0.00003);
    nh_.param<float>("imuGravity", imu_gravity_, 9.80511);
    nh_.param<std::vector<double>>("extrinsicRot", ext_rot_v_, std::vector<double>());
    nh_.param<std::vector<double>>("extrinsicRPY", ext_rpy_v_, std::vector<double>());
    nh_.param<std::vector<double>>("extrinsicTrans", ext_trans_v_, std::vector<double>());
    ext_rot_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_rot_v_.data(), 3, 3);
    ext_rpy_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_rpy_v_.data(), 3, 3);
    ext_trans_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_trans_v_.data(), 3, 1);
    ext_q_ = Eigen::Quaterniond(ext_rpy_);

    ROS_INFO("imuAccNoise: %f, imuGyrNoise: %f, imuAccBiasN: %f, imuGyrBiasN: %f",imu_acc_noise_, imu_gyr_noise_, imu_acc_biasN_, imu_gyr_biasN_);

    // 外参这里只考虑平移？？？
    // lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(ext_trans_.x(), ext_trans_.y(), ext_trans_.z()));
    // imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-ext_trans_.x(), -ext_trans_.y(), -ext_trans_.z()));
    lidar2Imu = gtsam::Pose3(gtsam::Rot3(ext_q_.w(), ext_q_.x(), ext_q_.y(), ext_q_.z()), gtsam::Point3(ext_trans_.x(), ext_trans_.y(), ext_trans_.z()));
    imu2Lidar = lidar2Imu.inverse();
    

    nh_.param<std::string>("imu_topic", imu_topic_, "/imu_raw");
    nh_.param<std::string>("lidar_odom_topic_sub", lidar_odom_topic_sub_, "/lidar_mapping/odometry");
    nh_.param<std::string>("imu_odom_topic_pub", imu_odom_topic_pub_, "/odometry/imu_incremental");
    nh_.param<std::string>("odometry_frame_id", odometry_frame_, "odom");

    ROS_INFO("imu_topic: %s", imu_topic_.c_str());

    imu_sub_      = nh_.subscribe<sensor_msgs::Imu>  (imu_topic_, 2000, &IMUPreintegration::imuCallback, this, ros::TransportHints().tcpNoDelay());
    odometry_sub_ = nh_.subscribe<nav_msgs::Odometry>(lidar_odom_topic_sub_, 5, &IMUPreintegration::odometryCallback, this, ros::TransportHints().tcpNoDelay());

    imu_odometry_pub_ = nh_.advertise<nav_msgs::Odometry> (imu_odom_topic_pub_, 2000);

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imu_gravity_);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imu_acc_noise_, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imu_gyr_noise_, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

    prior_pose_noise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    prior_vel_noise_   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
    prior_bias_noise_  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correction_noise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correction_noise2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
    noise_model_between_bias_ = (gtsam::Vector(6) << imu_acc_biasN_, imu_acc_biasN_, imu_acc_biasN_, imu_gyr_biasN_, imu_gyr_biasN_, imu_gyr_biasN_).finished();
    
    imu_integrator_opt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    imu_integrator_imu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        

}


/**
 * @function: 重置isam2优化相关求解器，因子图模型，顶点值
 * @param {*}
 * @return {*}
 */
void IMUPreintegration::resetOptimization() {
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer_ = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newgraph_factors_;
    graph_factors_ = newgraph_factors_;

    gtsam::Values Newgraph_values_;
    graph_values_ = Newgraph_values_;
}


/**
 * @function: 重置参数变量：上次imu消息时间戳，lidar odometry优化完成标志位，系统初始化标志位
 * @param {*}
 * @return {*}
 */
void IMUPreintegration::resetParams() {
    lastImuT_imu_ = -1;
    first_opt_done_ = false;
    system_initialized_ = false;
}

bool IMUPreintegration::failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur){
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30)
    {
        ROS_WARN("Large velocity, reset IMU-preintegration!");
        return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        ROS_WARN("Large bias, reset IMU-preintegration!");
        return true;
    }

    return false;
}

/**
 * @function: 将imu原始数据转换到lidar坐标系
 * @param {sensor_msgs::Imu}
 * @return {sensor_msgs::Imu}
 */
sensor_msgs::Imu IMUPreintegration::imuConverter(const sensor_msgs::Imu& imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = ext_rot_ * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = ext_rot_ * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    // Eigen四元数是Hamilton convention 需要右乘四元数
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * ext_q_;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}


void IMUPreintegration::odometryCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    std::lock_guard<std::mutex> lock(mtx_);

    // 当前lidar odom时间戳
    double currentCorrectionTime = ROS_TIME(odomMsg);

    // 确保有收到imu数据，否则直接返回
    if (imu_que_opt_.empty())
        return;

    // 当前lidar位姿
    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;
    bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


    // 0. 系统初始化
    if (system_initialized_== false)
    {
        // 重置优化
        resetOptimization();

        // 丢掉老的imu数据，跟当前lidar odom时间戳对齐
        while (!imu_que_opt_.empty())
        {
            if (ROS_TIME(&imu_que_opt_.front()) < currentCorrectionTime - delta_t)
            {
                // 更新上一帧imu数据时间戳
                lastImuT_opt_ = ROS_TIME(&imu_que_opt_.front());
                imu_que_opt_.pop_front();
            }
            else
                break;
        }
        // step1: 因子图模型加入先验因子

        // 通过lidar位姿初始化当前时刻imu初始位姿
        // T_w_imu = T_w_velo * T_velo_imu
        prev_pose_ = lidarPose.compose(lidar2Imu);
        // 因子图添加一个位姿的先验因子
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prev_pose_, prior_pose_noise_);
        graph_factors_.add(priorPose);
        // 初始速度为0? 应该可以通过轮速计或者gps获取到当前时刻的速度更好
        prev_vel_ = gtsam::Vector3(0, 0, 0);
        // 因子图添加一个速度的先验因子
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prev_vel_, prior_vel_noise_);
        graph_factors_.add(priorVel);
        // 因子图添加一个bias的先验因子
        prev_bias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prev_bias_, prior_bias_noise_);
        graph_factors_.add(priorBias);

        // step2: 插入顶点值作为初始值
        graph_values_.insert(X(0), prev_pose_);
        graph_values_.insert(V(0), prev_vel_);
        graph_values_.insert(B(0), prev_bias_);

        // step3: 执行1次优化
        optimizer_.update(graph_factors_, graph_values_);

        // step4: reset
        graph_factors_.resize(0);
        graph_values_.clear();

        // 为imu预积分器设置初始bias
        imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_);
        imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);

        // 初始化成功，直接返回
        key = 1;
        system_initialized_ = true;
        return;
    }


    // 滑窗因子图优化，第100次重置优化相关，添加先验因子
    if (key == 100)
    {
        // 得到边缘协方差
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key-1)));
        // 重置因子图优化
        resetOptimization();
        // 添加pose先验因子
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prev_pose_, updatedPoseNoise);
        graph_factors_.add(priorPose);
        // 添加速度先验因子
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prev_vel_, updatedVelNoise);
        graph_factors_.add(priorVel);
        // 添加bias先验因子
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prev_bias_, updatedBiasNoise);
        graph_factors_.add(priorBias);
        // 插入顶点值作为初始值
        graph_values_.insert(X(0), prev_pose_);
        graph_values_.insert(V(0), prev_vel_);
        graph_values_.insert(B(0), prev_bias_);
        // 执行一次优化
        optimizer_.update(graph_factors_, graph_values_);
        graph_factors_.resize(0);
        graph_values_.clear();

        // 优化次数变量key重新从1开始计数
        key = 1;
    }

    // 1. 对两次因子图优化之间的imu数据进行预积分
    while (!imu_que_opt_.empty())
    {
        // pop and integrate imu data that is between two optimizations
        sensor_msgs::Imu *thisImu = &imu_que_opt_.front();
        double imuTime = ROS_TIME(thisImu);
        if (imuTime < currentCorrectionTime - delta_t)
        {
            double dt = (lastImuT_opt_ < 0) ? (1.0 / 100.0) : (imuTime - lastImuT_opt_);
            imu_integrator_opt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            
            lastImuT_opt_ = imuTime;
            imu_que_opt_.pop_front();
        }
        else
            break;
    }

    // 添加imu预积分因子到因子图
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_opt_);
    // 前一时刻的imu的位姿和速度;当前时刻imu的位姿，速度;前一时刻的imu bias估计
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graph_factors_.add(imu_factor);
    // 添加两次优化之间的bias因子，默认是两次优化之间bias保持不变 
    graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_opt_->deltaTij()) * noise_model_between_bias_)));
    // 添加pose因子：当前测量值和估计值应该相同
    // T_w_imu = T_w_velo * T_velo_imu
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    // 如果是退化,给一个较大的协方差矩阵
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correction_noise2_ : correction_noise_);
    graph_factors_.add(pose_factor);
    // 利用前一时刻的状态和bias预测当前时刻的状态
    gtsam::NavState propState_ = imu_integrator_opt_->predict(prev_state_, prev_bias_);
    // 顶点插入预测位姿，预测速度以及bias作为初始值
    graph_values_.insert(X(key), propState_.pose());
    graph_values_.insert(V(key), propState_.v());
    graph_values_.insert(B(key), prev_bias_);
    // 执行两次优化
    optimizer_.update(graph_factors_, graph_values_);
    optimizer_.update();
    graph_factors_.resize(0);
    graph_values_.clear();
    // 拿到优化后的结果，更新状态变量为下一阶段做准备
    gtsam::Values result = optimizer_.calculateEstimate();
    prev_pose_  = result.at<gtsam::Pose3>(X(key));
    prev_vel_   = result.at<gtsam::Vector3>(V(key));
    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
    prev_bias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // 为用于优化的imu预积分器重置bias
    imu_integrator_opt_->resetIntegrationAndSetBias(prev_bias_);
    // 检测是否需要重置
    if (failureDetection(prev_vel_, prev_bias_))
    {
        // 如果优化失败，重置系统
        resetParams();
        return;
    }


    // 2. 优化后,重新将用于正常imu预积分的imu数据进行积分
    prev_state_odom_ = prev_state_;
    prev_bias_odom_  = prev_bias_;
    // 首先丢弃掉比当前矫正bias的时间更早的imu数据
    double lastImuQT = -1;
    while (!imu_que_imu_.empty() && ROS_TIME(&imu_que_imu_.front()) < currentCorrectionTime - delta_t)
    {
        lastImuQT = ROS_TIME(&imu_que_imu_.front());
        imu_que_imu_.pop_front();
    }
    // 再使用最新优化的bias对imu数据进行积分
    if (!imu_que_imu_.empty())
    {
        // reset bias use the newly optimized bias
        imu_integrator_imu_->resetIntegrationAndSetBias(prev_bias_odom_);
        // 从开始优化的时刻一直开始积分
        for (int i = 0; i < (int)imu_que_imu_.size(); ++i)
        {
            sensor_msgs::Imu *thisImu = &imu_que_imu_[i];
            double imuTime = ROS_TIME(thisImu);
            double dt = (lastImuQT < 0) ? (1.0 / 100.0) :(imuTime - lastImuQT);

            imu_integrator_imu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            lastImuQT = imuTime;
        }
    }
    // 因子图优化次数加1
    ++key;
    // 第一次优化完成
    first_opt_done_ = true;
}


void IMUPreintegration::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_raw) {
    // 数据加锁
    std::lock_guard<std::mutex> lock(mtx_);
    // 将imu原始数据转换到lidar坐标系
    sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
    ROS_INFO("imuTime: %f", ROS_TIME(&thisImu));

    // 保存转换后的数据到队列
    imu_que_opt_.push_back(thisImu);
    imu_que_imu_.push_back(thisImu);

    // 如果没有优化的bias,直接返回
    if (first_opt_done_ == false)
        return;

    double imuTime = ROS_TIME(&thisImu);
    // ROS_INFO("imuTime: %f", imuTime);
    double dt = (lastImuT_imu_ < 0) ? (1.0 / 100.0) : (imuTime - lastImuT_imu_);
    lastImuT_imu_ = imuTime;

    // 前面odometryCallback已经对imu_que_imu_数据积分了，这里只对单帧imu数据积分得到最新的状态
    imu_integrator_imu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

    // 预测最新的状态
    gtsam::NavState currentState = imu_integrator_imu_->predict(prev_state_odom_, prev_bias_odom_);

    // 发布imu 预测的lidar odometry 用于融合lidar mapping的odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = odometry_frame_;
    odometry.child_frame_id = "odom_imu";

    // 得到imu位姿T_w_b
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    // 通过lidar到imu的外参得到lidar在世界坐标系下的位姿
    // T_w_velo = T_w_imu * T_imu_velo
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prev_bias_odom_.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prev_bias_odom_.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prev_bias_odom_.gyroscope().z();
    imu_odometry_pub_.publish(odometry);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu preintegration");

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

    ros::NodeHandle nh("~");

    IMUPreintegration IMUPreInte(nh);

    ros::spin();

    return 0;
}
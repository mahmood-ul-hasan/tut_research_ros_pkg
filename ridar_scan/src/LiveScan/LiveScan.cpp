#include "LiveScan/LiveScan.h"
#include <ctime>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <tf/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
// use tf2_ros::TransformBroadcaster
// #include <tf2_ros/transform_broadcaster.h>
#include "../../orion_rotating_base/include/orion_rotating_base/rotating_base.h"

LiveScan::LiveScan(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr),
    localNodeHandlePtr_(localNodeHandlePtr),
    normalizeRef_(false),
    refMin_(300.0),
    refMax_(900.0),
    exportPcd_(false),
    exportPts_(false),
    exportPly_(false),
    meshThresh_(1.0),
    enableTiltCompensation_(true),
    calibPan0_(M_PI_2),
    calibPan1_(M_PI_2),
    calibTilt1_(0.0),
    isMeasuring_(false)
{
    localNodeHandlePtr_->param("is_simulation", isSimulation_, false);
    // Initialize Subscribers and Publishers
    if (isSimulation_)
    {
        scanRange_ = SCAN_RANGE_SIM;
        nowYawSim_ = -SCAN_RANGE_SIM;
        // laserScanSub_.subscribe(*nodeHandlePtr_, "/front_laser_link/scan", 2000);
        // rotatingBaseStateSub_.subscribe(*nodeHandlePtr_, "/model2scan_node/model_joint_states", 2000);
        // poseSub_.subscribe(*nodeHandlePtr_, "/model2scan_node/model_pose", 2000);
        // syncSim_         = new message_filters::Synchronizer<SyncPolicySim>(SyncPolicySim(10), laserScanSub_, rotatingBaseStateSub_, poseSub_);
        // syncSim_->registerCallback(boost::bind(&LiveScan::scanCallback, this, _1, _2, _3));
        laserScanSub_.subscribe(*nodeHandlePtr_, "/front_laser_link/scan", 2000);
        rotatingBaseStateSub_.subscribe(*nodeHandlePtr_, "/sensor_model_respawner_node/joint_angle", 4000);
        // laserScanSubSim_ = nodeHandlePtr_->subscribe("/front_laser_link/scan", 1, &LiveScan::callbackSim, this);
        jointPubSim_  = nodeHandlePtr_->advertise<sensor_msgs::JointState>("/ridar_scan/jointMsg", 1);
    } else {
        scanRange_ = SCAN_RANGE;
        laserScanSub_.subscribe(*nodeHandlePtr_, "/scan", 2000);
        rotatingBaseStateSub_.subscribe(*nodeHandlePtr_, "/orion_rotating_base/joint_states", 4000);
        // syncNotSim_         = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), laserScanSub_, rotatingBaseStateSub_);
        // syncNotSim_->registerCallback(boost::bind(&LiveScan::scanCallback, this, _1, _2));
    }
    sync_         = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), laserScanSub_, rotatingBaseStateSub_);
    sync_->registerCallback(boost::bind(&LiveScan::scanCallback, this, _1, _2));
    
    liveScanPub_  = localNodeHandlePtr_->advertise<sensor_msgs::PointCloud2>("cloud", 1);
    keyScanPub_   = nodeHandlePtr_->advertise<pointcloud_integrator::keyScanMsg>("/ridar_scan/liveScan", 1);

    // Initialize ServiceClients and ServiceServers
    respawnClient_ = nodeHandlePtr_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    rotatingBaseActionClient_ = nodeHandlePtr_->serviceClient<orion_rotating_base::sendCommand>("orion_rotating_base/cmd");
    executeLiveScanSrv_ = localNodeHandlePtr_->advertiseService("execute_live_scan", &LiveScan::executeLiveScanSrv, this);
  
    // Initialize member variable
    actionNum_ = 0;
    isRotatingBaseWork_ = false;

    // Set parameters
    std::string defaultModelName = "dtw_robot";
    std::string defaultReferenceFrame = "world";
    std::string defaultSensorModelFrameId = "lidar_sensor_model";
    localNodeHandlePtr_->param("rotation_speed", rotationSpeed_, RotatingBase::MAX_SPEED);
    localNodeHandlePtr_->param("ref_normalization", normalizeRef_, normalizeRef_);
    localNodeHandlePtr_->param("ref_min", refMin_, refMin_);
    localNodeHandlePtr_->param("ref_max", refMax_, refMax_);
    localNodeHandlePtr_->param("export_pcd", exportPcd_, exportPcd_);
    localNodeHandlePtr_->param("export_pts", exportPts_, exportPts_);
    localNodeHandlePtr_->param("export_ply", exportPly_, exportPly_);
    localNodeHandlePtr_->param("mesh_thresh", meshThresh_, meshThresh_);
    localNodeHandlePtr_->param("enable_tilt_compensation", enableTiltCompensation_, enableTiltCompensation_);
    localNodeHandlePtr_->param("is_offline", isOffLine_, false);
    localNodeHandlePtr_->param("rotate_cycle", rotateCycle_, 0.1);
    localNodeHandlePtr_->param("sensor_model_name", sensorModelName_, defaultModelName);
    localNodeHandlePtr_->param("reference_frame", referenceFrame_, defaultReferenceFrame);
    localNodeHandlePtr_->param("sensor_model_name", sensorModelFrameId_, defaultSensorModelFrameId);

    readCalibFile();

    // moveRotatingBase(10, 180.0);
}


LiveScan::~LiveScan()
{
    delete sync_;
    // delete syncSim_;
}

void LiveScan::readCalibFile()
{
    if (!enableTiltCompensation_) {
        return;
    }

    std::ifstream ifs(PACKAGE_PATH + "/data/calibration.csv");
    if (!ifs) {
        ROS_INFO_STREAM("calibration file not found");
        ROS_INFO_STREAM("tilt compensation disabled");
        enableTiltCompensation_ = false;

        return;
    }

    std::string token;
    ifs >> token >> calibPan0_
        >> token >> calibPan1_
        >> token >> calibTilt1_;

    ROS_INFO_STREAM("calibration file loaded");
    ROS_INFO_STREAM("calib_pan0:" << calibPan0_);
    ROS_INFO_STREAM("calib_pan1:" << calibPan1_);
    ROS_INFO_STREAM("calib_tilt1:" << calibTilt1_);
}

bool LiveScan::initPositionRotatingBase()
{
    if(!moveRotatingBase(RotatingBase::MAX_SPEED, (cworccw_ == CLOCK_WISE ? 1 : -1) * (-scanRange_ - MOVE_MARGIN))) {
        ROS_ERROR("Failed to call service! at LiveScan::initPositionRotatingBase()");
        return false;
    }
    return true;
}


void LiveScan::dataReset()
{
    laserScanData_.clear();
    rotatingBaseStateData_.clear();
    // pointAndQuat_.clear();
    isRotatingBaseWork_ = false;
}


bool LiveScan::executeScan(int speed)
{
    if (!isOffLine_)
    {
        // size_t rotatingBaseDataSize = rotatingBaseStateData_.size();
        // while (rotatingBaseStateData_.size() == rotatingBaseDataSize) {
        //     ros::spinOnce();  // scanCallbackを実行
        // }
        double nowPos;
        if (isSimulation_)
        {
            nowPos = nowYawSim_;
        } else {
            ROS_INFO_STREAM("rotatingBaseStateData_.size() = " << rotatingBaseStateData_.size());
            nowPos = rotatingBaseStateData_.back().position[0];
        }
        // if (isSimulation_)
        // {
        //     // nowYawSim_ = nowPos;
        //     // nowPoseSim_ = pointAndQuat_.back();
        // }
        
        if (nowPos < 0)
        {
            ROS_INFO("clock_wise");
            cworccw_ = CLOCK_WISE;
        } else {
            ROS_INFO("counter_clock_wise");
            cworccw_ = COUNTER_CLOCKWISE;
        }
        // rotatingBaseを初期位置に移動
        if (   (cworccw_ == CLOCK_WISE && (nowPos < -scanRange_ - MOVE_MARGIN*1.5 || nowPos > -scanRange_ - MOVE_MARGIN*0.5))
            || (cworccw_ == COUNTER_CLOCKWISE && (nowPos < scanRange_ + MOVE_MARGIN*0.5 || nowPos > scanRange_ + MOVE_MARGIN*1.5)))
        {
            ROS_INFO("initPositionRotatingBase()");
            initPositionRotatingBase();
        }
    }

    dataReset(); // データを初期化

    panStart_ = (cworccw_ == CLOCK_WISE ? -1 : 1) * (scanRange_ + MOVE_MARGIN);
    panEnd_ = (cworccw_ == CLOCK_WISE ? 1 : -1) * (scanRange_ + MOVE_MARGIN);
    progressMeasuring_ = 0.0;
    progressUnit_ = 0.05;
    isMeasuring_ = true;

    std::thread scanThread(&LiveScan::receiveScanDataThread, this);

    if (!isOffLine_)
    {
        mutex_.lock();
        ROS_INFO("mesurement progress: 0%%");
        if (isSimulation_)
        {
            isRotatingBaseWork_ = true;
        }
        mutex_.unlock();
        if (!moveRotatingBase(speed, (cworccw_ == CLOCK_WISE ? 1 : -1) * (scanRange_ + MOVE_MARGIN))) return false;
        if (isSimulation_)
        {
            mutex_.lock();
            isRotatingBaseWork_ = false;
            mutex_.unlock();
        }
    } else {
        isRotatingBaseWork_ = true;

        ROS_INFO("Please press Enter key when rosbag play finishes.");
        // Enterが押されるまで待つ
        while(std::cin.get() != '\n')
        {
        }

        isRotatingBaseWork_ = false;
    }
    
    scanThread.join();

    isMeasuring_ = false;
    if (!isOffLine_)
    {
        mutex_.lock();
        ROS_INFO("mesurement progress: 100%%");
        mutex_.unlock();
    }
  
    ROS_INFO_STREAM("Scan list size is " << laserScanData_.size());
    
    if (isOffLine_)
    {
        if (rotatingBaseStateData_[0].position[0] < rotatingBaseStateData_.back().position[0])
        {
            cworccw_ = CLOCK_WISE;
        } else {
            cworccw_ = COUNTER_CLOCKWISE;
        }
    }

    // 計測開始と終了時のインデックスを取得
    size_t scanStart = rotatingBaseStateData_.size() + 1, scanEnd = rotatingBaseStateData_.size();
    for (size_t i = 0; i < rotatingBaseStateData_.size(); ++i) {
    // ROS_INFO_STREAM("ok16 rotatingBaseStateData_[" << i << "].position[0] = " << rotatingBaseStateData_[i].position[0] << ", scanRange_ = " << scanRange_);
        if (   (cworccw_ == CLOCK_WISE && rotatingBaseStateData_[i].position[0] < (isSimulation_ ? pcl::deg2rad(-scanRange_) : -scanRange_))
            || (cworccw_ == COUNTER_CLOCKWISE && rotatingBaseStateData_[i].position[0] > (isSimulation_ ? pcl::deg2rad(scanRange_) : scanRange_))) {
            scanStart = i;
            break;
        }
    }
    for (size_t i = rotatingBaseStateData_.size() - 1; i < rotatingBaseStateData_.size(); --i) {
    // ROS_INFO_STREAM("ok18 rotatingBaseStateData_[" << i << "].position[0] = " << rotatingBaseStateData_[i].position[0] << ", scanRange_ = " << scanRange_);
        if (   (cworccw_ == CLOCK_WISE && rotatingBaseStateData_[i].position[0] > (isSimulation_ ? pcl::deg2rad(scanRange_) : scanRange_))
            || (cworccw_ == COUNTER_CLOCKWISE && rotatingBaseStateData_[i].position[0] < (isSimulation_ ? pcl::deg2rad(-scanRange_) : -scanRange_))) {
            scanEnd = i;
            break;
        }
    }
    if (scanStart == rotatingBaseStateData_.size() + 1)
    {
        ROS_ERROR("scanStart is invalid value");
        return false;
    }
    if (scanEnd == rotatingBaseStateData_.size())
    {
        ROS_ERROR("scanEnd is invalid value");
        return false;
    }

  
    pcl::PointCloud<pcl::PointXYZI> keyScan;
    ScanPlans2PointCloud::generatePointCloud(keyScan,
                laserScanData_, rotatingBaseStateData_,
                scanStart, scanEnd,
                CANT_SCAN_DISTANCE, SCAN_DISTANCE_MAX, isSimulation_, (isSimulation_ ? SCAN_PLANE_DISTANCE_SIM : SCAN_PLANE_DISTANCE),
                enableTiltCompensation_, calibPan0_, calibPan1_, calibTilt1_,
                (isSimulation_ ? pcl::deg2rad(scanRange_) : scanRange_),
                normalizeRef_, refMin_, refMax_);
    // ////function generatePointCloud
    // // スキャンデータから変換されたある地点における3次元データを保持
    // pcl::PointCloud<pcl::PointXYZI> keyScan;
  
    // // SICKから得られたスキャンデータを割り振る処理
    // for (size_t i = scanStart; i <= scanEnd; ++i) {
    //     sensor_msgs::LaserScan laserScan = laserScanData_[i];
    //     sensor_msgs::JointState rotatingBaseState = rotatingBaseStateData_[i];
    
    //     // スキャンデータが取得された際のパン角を更新
    //     double panAngle = rotatingBaseState.position[0];
 
    //     // scanRange_以外の範囲で取得したスキャンデータは変換に考慮しない
    //     if (panAngle > (isSimulation_ ? pcl::deg2rad(scanRange_) : scanRange_) || panAngle < (isSimulation_ ? pcl::deg2rad(-1.0 * scanRange_) : -1.0 * scanRange_))
    //         continue;
    
    //     // SICKのスキャン開始位置[rad]および角度分解能[rad]の実測値を保持
    //     float urgStart, urgIncrement;
    //     urgStart = laserScan.angle_min;
    //     urgIncrement = laserScan.angle_increment;
    
    //     for (size_t j = 0; j < laserScan.ranges.size(); ++j) {
    //         // SICKのスキャン角度を更新
    //         float urgAngle = urgStart + urgIncrement * j;

    //         // 距離データと反射強度の値を取得
    //         float range = laserScan.ranges[j];
    //         float intensity = laserScan.intensities[j];
      
    //         // 距離データから3次元位置を計算
    //         pcl::PointXYZI point = range2Point(range, intensity, urgAngle, panAngle);

    //         keyScan.push_back(point);
    //     }
    // }

    // if (normalizeRef_) {
    //     // リフレクタンス値の正規化
    //     normalizeReflectance(keyScan, refMin_, refMax_);
    // }
    // //// function ends

    if (!isSimulation_)
    {
        for (auto& it : keyScan)
        {
            it.y = std::cos(SEMICIRCLE_RAD) * it.y - std::sin(SEMICIRCLE_RAD) * it.z;
            it.z = std::sin(SEMICIRCLE_RAD) * it.y + std::cos(SEMICIRCLE_RAD) * it.z; 
        }
    }

    if (exportPcd_) {
        savePcd(keyScan);
    }
 
    // ある地点における3次元スキャンデータをptsに変換
    Pts pts;
    pcl2Pts(keyScan, pts);
  
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    std::string time_string
        = "_" + std::to_string(pnow->tm_year + 1900) + "-" + std::to_string(pnow->tm_mon + 1) + "-" + std::to_string(pnow->tm_mday)
          + "_" + std::to_string(pnow->tm_hour) + "-" + std::to_string(pnow->tm_min) + "-" + std::to_string(pnow->tm_sec);

    if (exportPts_) {
        std::string fileName = PACKAGE_PATH + fileName_ + time_string + ".pts";
        if (pts.savePtsFile(fileName))  ROS_INFO_STREAM("Succeed at a pts file output");
    }
  
    if (exportPly_) {
        Ply ply;
        pts.convertToPly(ply, meshThresh_);  
        std::string fileName = PACKAGE_PATH + fileName_ + time_string + ".ply";
        if (ply.savePlyFile(fileName, ply::BINARY))  ROS_INFO_STREAM("Succeed at a ply file output");
    }
  
    sensor_msgs::PointCloud2 result;
    pcl::toROSMsg(keyScan, result);

    // ある地点における3次元スキャンデータをPointCloud2メッセージでpublish
    result.header.frame_id = "laser";
    result.header.stamp = ros::Time::now();
    liveScanPub_.publish(result);
  
    // ある地点における3次元スキャンデータをkeyScanMsgに変換しpublish
    pointcloud_integrator::keyScanMsg keyScanMsg;
    pcl2KeyScanMsg(keyScan, keyScanMsg, actionNum_, normalizeRef_);
    keyScanPub_.publish(keyScanMsg);
  
    actionNum_++;
  
    return true;
}


void LiveScan::receiveScanDataThread()
{
    mutex_.lock(); // ここでスレッド間による共通の変数をロック

    bool isFirstLoop = true;
    while (isRotatingBaseWork_) {
        if (isFirstLoop)
        {
            mutex_.unlock();
            isFirstLoop = false;
        }
        ros::spinOnce();  // scanCallback or callbackSimを実行
    }
}


void LiveScan::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg, const sensor_msgs::JointState::ConstPtr& rotatingBaseMsg)
{
    mutex_.lock();
    laserScanData_.push_back(*scanMsg);
    rotatingBaseStateData_.push_back(*rotatingBaseMsg);
    mutex_.unlock();
    showProgress();
}

// void LiveScan::callbackSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
// {
//     mutex_.lock();
//     laserScanData_.push_back(*scanMsg);
//     // rotatingBaseStateData_.push_back(*rotatingBaseMsg);
//     sensor_msgs::JointState rotatingBaseMsg;
//     rotatingBaseMsg.position.resize(1);
//     rotatingBaseMsg.position[0] = pcl::deg2rad(nowYawSim_);
//     rotatingBaseStateData_.push_back(rotatingBaseMsg);
//     // ROS_INFO_STREAM("panStart_: " << panStart_ << "panEnd_: " << panEnd_);
//     // ROS_INFO_STREAM("nowYawSim_: " << nowYawSim_);
//     mutex_.unlock();
//     showProgress();
// }

void LiveScan::showProgress()
{
    mutex_.lock();
    if (isMeasuring_)
    {
        double progressTemp = (rotatingBaseStateData_.back().position[0] - (isSimulation_ ? pcl::deg2rad(panStart_) : panStart_)) / (isSimulation_ ? pcl::deg2rad(panEnd_ - panStart_) : panEnd_ - panStart_);
        if (progressTemp > progressMeasuring_ + progressUnit_)
        {
            progressMeasuring_ += progressUnit_;
            ROS_INFO_STREAM("mesurement progress: " << (progressMeasuring_ * 100) << "%");
        }
    }
    mutex_.unlock();
}

void LiveScan::quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

bool LiveScan::moveRotatingBase(int speed, double endPos)
{
    if (isSimulation_)
    {
        int spdSim = speed;
        if (speed <= 0)
        {
            spdSim = 10;
        }
    
        // static tf2_ros::Buffer tfBuffer;
        // static tf2_ros::TransformListener tfListener(tfBuffer);
        // size_t rotatingBaseDataSize = rotatingBaseStateData_.size();
        // while (rotatingBaseStateData_.size() == rotatingBaseDataSize) {
        //     ros::spinOnce();  // scanCallbackを実行
        // }

        // double nowPosi = rotatingBaseStateData_.back().position[0];
        // geometry_msgs::Pose nowPose = pointAndQuat_.back();

        // double nowPosi = nowYawSim_;
        // geometry_msgs::Pose nowPose = nowPoseSim_;
        // geometry_msgs::TransformStamped nowTransformStamped;

        // double nowPosi = 0.0;
        // geometry_msgs::Pose nowPose;
        // nowPose.position.x = 0.0;
        // nowPose.position.y = 0.0;
        // nowPose.position.z = 0.0;
        // nowPose.orientation.x = 0.0;
        // nowPose.orientation.y = 0.0;
        // nowPose.orientation.z = 0.0;
        // nowPose.orientation.w = 1.0;

        mutex_.lock();
        double moveAngle = endPos - nowYawSim_;
        mutex_.unlock();
        size_t timesOfMove = static_cast<size_t>((moveAngle > 0 ? moveAngle : -moveAngle) / spdSim / rotateCycle_);
        // spdSim = 5 [deg/sec]
        // moveAngle = 15 [deg]
        // rotateCycle_ = 0.1 [sec]
        // timesOfMove = moveAngle[deg] / spdSim[deg/sec] / rotateCycle_[sec];
        // timesOfMove = 15 / 5 / 0.1;
        double moveAnglInCycle = moveAngle / timesOfMove;
        // // double nowYaw = nowYawSim_;
        // // double roll, pitch, yaw;
        // // quaternion2EulerAngles(roll, pitch, yaw, nowPose.orientation);
        // // geometry_msgs::Pose targetPose = nowPose;
        // geometry_msgs::Pose targetPose;
        for (size_t index = 0; index < timesOfMove; index++)
        {
        //     // geometry_msgs::TransformStamped nowTransformStamped;
        //     try{
        //         nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, ros::Time(0));
        //             // ↑referenceFrame_, sensorModelFrameId_ を逆にするとなぜかnowTransformStamped.transform.translation.zがおかしく（マイナスに）なる
        //     }
        //     catch (tf2::TransformException &ex) {
        //         ROS_WARN("%s",ex.what());
        //         ros::Duration(1.0).sleep();
        //         continue;
        //     }
        //     double roll, pitch, sensorYaw;
        //     quaternion2EulerAngles(roll, pitch, sensorYaw, nowTransformStamped.transform.rotation);
        //     // nowYaw += moveAnglInCycle;
            mutex_.lock();
            nowYawSim_ += moveAnglInCycle;
            mutex_.unlock();

            // Publish Position
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(1);
            joint_state.position.resize(1);
            joint_state.velocity.resize(1);
            joint_state.name[0] = "rotating_base [rad, rad/sec]";
            mutex_.lock();
            joint_state.position[0] = pcl::deg2rad(nowYawSim_);
            mutex_.unlock();
            joint_state.velocity[0] = pcl::deg2rad(static_cast<double>(spdSim));
            
            jointPubSim_.publish(joint_state);

            // static tf2_ros::TransformBroadcaster br;
            // geometry_msgs::TransformStamped transformStamped;
            
            // transformStamped.header.stamp = ros::Time::now();
            // transformStamped.header.frame_id = "boom_link";
            // transformStamped.child_frame_id = sensorModelFrameId_;

            // // transformStamped.transform.translation.x = linkMsg->pose[index].position.x;
            // // transformStamped.transform.translation.y = linkMsg->pose[index].position.y;
            // // transformStamped.transform.translation.z = linkMsg->pose[index].position.z;
            // transformStamped.transform.translation.x = 15.0;
            // transformStamped.transform.translation.y = 0.0;
            // transformStamped.transform.translation.z = 0.0;
            // tf2::Quaternion q;
            // q.setRPY(0.0, 0.0, nowYawSim_);
            // transformStamped.transform.rotation.x = q.x();
            // transformStamped.transform.rotation.y = q.y();
            // transformStamped.transform.rotation.z = q.z();
            // transformStamped.transform.rotation.w = q.w();

            // br.sendTransform(transformStamped);


            // geometry_msgs::Quaternion targetQuat = eulerAngles2Quaternion(roll, pitch, sensorYaw + pcl::deg2rad(nowYawSim_));
            // // geometry_msgs::Quaternion targetQuat = eulerAngles2Quaternion(roll, pitch, pcl::deg2rad(nowYaw));
            // // targetPose = nowPose;
            // targetPose.position.x = nowTransformStamped.transform.translation.x;
            // targetPose.position.y = nowTransformStamped.transform.translation.y;
            // targetPose.position.z = nowTransformStamped.transform.translation.z;
            // // ROS_INFO_STREAM("targetPose.position.x = " << targetPose.position.x);
            // // ROS_INFO_STREAM("targetPose.position.y = " << targetPose.position.y);
            // // ROS_INFO_STREAM("targetPose.position.z = " << targetPose.position.z);
            // // ROS_INFO("-------------------------");
            // targetPose.orientation = targetQuat;
            // if (!moveSensorModel(sensorModelName_, referenceFrame_, targetPose))
            // {
            //     return false;
            // }
            ros::Duration(rotateCycle_).sleep();
        }

        // nowYawSim_ = nowYaw;
        // nowPoseSim_ = targetPose;
        return true;
    } else {
        orion_rotating_base::sendCommand cmd;
        cmd.request.end_pos        = endPos;
        cmd.request.speed          = (speed == 0 ? rotationSpeed_ : speed);

        // 以下は実際にサービスで回転台を動作させている
        mutex_.lock();
        isRotatingBaseWork_ = true;
        mutex_.unlock();
    
        if (rotatingBaseActionClient_.call(cmd)) {
            mutex_.lock();
            isRotatingBaseWork_ = false;
            mutex_.unlock();

            ROS_INFO("Service call successful.");
            return true;
        }
        else {
            mutex_.lock();
            isRotatingBaseWork_ = false;
            mutex_.unlock();

            ROS_ERROR("Failed to call service! at LiveScan::moveRotatingBase()");
            return false;
        }
    }
}

geometry_msgs::Quaternion LiveScan::eulerAngles2Quaternion(double roll, double pitch, double yaw) const
{
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}


bool LiveScan::executeLiveScanSrv(ridar_scan::ExecuteLiveScanRequest& req, ridar_scan::ExecuteLiveScanResponse& res)
{
    if (req.file_name != "") {
        fileName_ = "/scans/" + req.file_name;
    }
    else {
        fileName_ = "/scans/LiveScan";
    }

    if (executeScan(req.speed)) {
        res.success = true;
    }
    else {
        res.success = false;
        ROS_ERROR("Error in live_scan executeLiveScanSrv function.");
    }

    return true;
}


void LiveScan::pcl2KeyScanMsg(const pcl::PointCloud<pcl::PointXYZI>& point, pointcloud_integrator::keyScanMsg& ptsMsg, const int& id, bool normalizeRef)
{
    ptsMsg.id                   = id;
    ptsMsg.time                 = ros::Time::now();
    ptsMsg.isAdditionalFrame    = false;
    ptsMsg.is8bitNormalized     = normalizeRef;

    ptsMsg.height   =   point.size() / ONE_SCAN_LINE_NUM;
    ptsMsg.width    =   ONE_SCAN_LINE_NUM;
  
    Eigen::Matrix4f poseMat;
    poseMat << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
    Sophus::Sim3f poseSim3(poseMat);  // scaled rotation, translation
    memcpy(ptsMsg.camToWorld.data(), poseSim3.data(), 7 * sizeof(float));
  
    ptsMsg.pointcloud.reserve(ptsMsg.height * ptsMsg.width);
    for (long idx = 0; idx < ptsMsg.width * ptsMsg.height; ++idx) {
        pointcloud_integrator::scanMsg tmp;
        tmp.point[0] = point[idx].x;
        tmp.point[1] = point[idx].y;
        tmp.point[2] = point[idx].z;

        float intensity = point[idx].intensity;
        tmp.color[0] = tmp.color[1] = tmp.color[2] = tmp.color[3] = intensity;

        ptsMsg.pointcloud.emplace_back(tmp);
    }
  
    ROS_INFO_STREAM("ScanMsg:" << ptsMsg.width << "x" << ptsMsg.height << "=" << ptsMsg.pointcloud.size() << " points.");
}

void LiveScan::pcl2Pts(const pcl::PointCloud<pcl::PointXYZI>& cloud, Pts& pts)
{
    // SICKから一回のスキャンで得られるデータのサイズおよびスキャン回数
    const int SCAN_NUM = cloud.size() / ONE_SCAN_LINE_NUM;
  
    pts.row() = SCAN_NUM * 2;
    pts.column() = ONE_SCAN_LINE_NUM / 2 + 1;
  
    std::vector<Eigen::Vector3d>& vertex = pts.vertex();
    std::vector<double>& reflectance = pts.reflectance();
  
    vertex.resize(pts.row() * pts.column());
    reflectance.resize( pts.row() * pts.column());
  
    size_t idx = 0;
  
    for (int row = 0; row < pts.row(); ++row) {
        for (int col = 0; col < pts.column(); ++col) {
            size_t _idx;
            if (row < SCAN_NUM)  _idx = ONE_SCAN_LINE_NUM * row + ONE_SCAN_LINE_NUM - 1 - col;
            else                 _idx = ONE_SCAN_LINE_NUM * (row - SCAN_NUM) + col;
	
            vertex[idx].x()  = cloud[_idx].x;
            vertex[idx].y()  = cloud[_idx].y;
            vertex[idx].z()  = cloud[_idx].z;
            reflectance[idx] = cloud[_idx].intensity;
            ++idx;
        }
    }
}


void LiveScan::savePcd(const pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    time_t now = time(NULL);
    struct tm *pnow = localtime(&now);
    std::string time_string
        = "_" + std::to_string(pnow->tm_year + 1900) + "-" + std::to_string(pnow->tm_mon + 1) + "-" + std::to_string(pnow->tm_mday)
          + "_" + std::to_string(pnow->tm_hour) + "-" + std::to_string(pnow->tm_min) + "-" + std::to_string(pnow->tm_sec);

    std::string path = PACKAGE_PATH + fileName_ + time_string + ".pcd";
    pcl::io::savePCDFileBinary(path, cloud);
}


void LiveScan::saveImage(Pts& pts)
{
    cv::Mat& rangeImage = pts.rangeImage();
    cv::Mat& refImage   = pts.refImage();
  
    rangeImage.create(pts.row(), pts.column(), CV_32F);
    refImage.create(pts.row(), pts.column(), CV_32F);
  
    size_t idx = 0;
    for (int row = 0; row < pts.row(); ++row) {
        for (int col = 0; col < pts.column(); ++col) {
            if (std::isnan(pts.reflectance()[idx])) {
                rangeImage.at<float>(row, col) = 0.0;
                refImage.at<float>(row, col)   = 0.0;
            }
            else {
                rangeImage.at<float>(row, col) = sqrt(pts.vertex()[idx].norm());
                refImage.at<float>(row, col)   = pts.reflectance()[idx];
            }
            ++idx;
        }
    }
  
    std::string path = PACKAGE_PATH + fileName_ + std::to_string(actionNum_);
    pts.save8bitImages(path);
}


void LiveScan::normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>& cloud, const double refMin, const double refMax)
{
    for (size_t idx = 0; idx < cloud.size(); ++idx) {
        float intensity = cloud[idx].intensity;

        // Scale intensity values from 0 to 1
        if (intensity < refMin)       intensity = 0.0;
        else if (intensity > refMax)  intensity = 1.0;
        else                          intensity = (intensity - refMin) / (refMax - refMin);
    
        cloud[idx].intensity = intensity;
    }
}


// pcl::PointXYZI LiveScan::range2Point(const float range, const float intensity, const float urgAngle, const double panAngle)
// {
//     pcl::PointXYZI point;
//     double panAngRad = (isSimulation_ ? panAngle : pcl::deg2rad(panAngle));

//     // 距離データの値が範囲外である場合
//     if (range <= CANT_SCAN_DISTANCE || range > SCAN_DISTANCE_MAX) {
//         // 計測することができなかったデータとして全ての値をNANとする
//         point.x = NAN;
//         point.y = NAN;
//         point.z = NAN;
//         point.intensity = NAN;

//         return point;
//     }

//     double xForUv = (isSimulation_ ? SCAN_PLANE_DISTANCE_SIM : range * cos(urgAngle));
//     double yForUv = (isSimulation_ ? range * sin(urgAngle) : SCAN_PLANE_DISTANCE);
//     double zForUv = (isSimulation_ ? -range * cos(urgAngle) : -range * sin(urgAngle));
//     Eigen::Vector3d uv0(xForUv, yForUv, zForUv);
//     Eigen::AngleAxisd rotation = Eigen::AngleAxisd(panAngRad, Eigen::Vector3d::UnitZ());

//     if(enableTiltCompensation_) {
//         double p = (panAngRad - calibPan0_) / (calibPan1_ - calibPan0_);
//         double tilt = (1 - p) * calibTilt1_;
//         rotation = Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()) * rotation;
//     }

//     Eigen::Vector3d xyz = rotation * uv0;

//     point.x = xyz.x();
//     point.y = xyz.y();
//     point.z = xyz.z();
//     point.intensity = intensity;

//     return point;
// }

bool LiveScan::moveSensorModel(const std::string targetModel, const std::string referenceFrame, const geometry_msgs::Pose& pose)
{
    // Set request value (For clarify)
    // std::string targetModel    = req.target_model;
    // std::string referenceFrame = req.frame_id;

    gazebo_msgs::SetModelState setModelState;
    gazebo_msgs::ModelState    modelState;

    // Set ModelState INFO
    modelState.model_name      = targetModel;
    // modelState.pose            = req.pose;
    modelState.pose            = pose;
    modelState.reference_frame = referenceFrame;
    
    // Servcie Call
    setModelState.request.model_state = modelState;
    size_t jndex = 0;
    while ( !respawnClient_.call(setModelState))
    {
        jndex++;
        if (jndex >= NUM_OF_RETRY)
        {
            ROS_ERROR("failed to connect");
            return false;
        }
    }

    return true;
}

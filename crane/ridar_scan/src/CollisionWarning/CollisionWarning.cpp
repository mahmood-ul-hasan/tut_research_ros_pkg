#include "CollisionWarning/CollisionWarning.h"
#include <vector>
#include <float.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>

CollisionWarning::CollisionWarning(ros::NodeHandle* nodeHandlePtr, bool isSimulation)
  : nodeHandlePtr_(nodeHandlePtr),
    isSimulation_(isSimulation),
    nowAngleSim_(0.0),
    isClockWise_(true),
    isFirst_(true),
    isFirstScan_(true),
    willPablishingCloud_(true)
{//ROS_INFO_STREAM("__LINE__ = " << __LINE__);
    // Initialize Subscribers and Publishers
    // jointSub_ = nodeHandlePtr->subscribe("/k_crane/joint_states", 1, &CollisionWarning::callback, this);
    if (isSimulation_)
    {
        laserScanSub_.subscribe(*nodeHandlePtr_, "/front_laser_link/scan", 2000);
        rotatingBaseStateSub_.subscribe(*nodeHandlePtr_, "/sensor_model_respawner_node/joint_angle", 1000);
        jointSub_.subscribe(*nodeHandlePtr_, "/k_crane/joint_states", 4000);
        syncSim_ = new message_filters::Synchronizer<SyncPolicySim>(SyncPolicySim(1000), laserScanSub_, jointSub_, rotatingBaseStateSub_);
        syncSim_->registerCallback(boost::bind(&CollisionWarning::callbackSim, this, _1, _2, _3));
        jointPubSim_ = nodeHandlePtr_->advertise<sensor_msgs::JointState>("jointMsg", 1);
        liveScanPub_ = nodeHandlePtr_->advertise<sensor_msgs::PointCloud2>("cloud", 1);
        collisionWarningResultPub_ = nodeHandlePtr_->advertise<std_msgs::Bool>("warning_result", 1);
        respawnClient_ = nodeHandlePtr_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    } else
    {
        laserScanSub_.subscribe(*nodeHandlePtr_, "/scan", 2000);
        rotatingBaseStateSub_.subscribe(*nodeHandlePtr_, "/orion_rotating_base/joint_states", 1000);
        syncNotSim_ = new message_filters::Synchronizer<SyncPolicyNotSim>(SyncPolicyNotSim(1000), laserScanSub_, rotatingBaseStateSub_);
        syncNotSim_->registerCallback(boost::bind(&CollisionWarning::callbackNotSim, this, _1, _2));
        rotatingBaseActionClient_ = nodeHandlePtr_->serviceClient<orion_rotating_base::sendCommand>("/orion_rotating_base/cmd");
        // liveScanPub_ = nodeHandlePtr_->advertise<sensor_msgs::PointCloud2>("cloud", 1);
    }

    resetVectors();

    std::string defaultReferenceFrame = "world";
    std::string defaultSensorModelFrameId = "lidar_sensor_model";
    std::string defaultRope1LinkModelFrameId = "rope1_link";
    std::string defaultWarningVisModelName = "warning_vis_box";
    nodeHandlePtr_->param("boom_length", boomLength_, 31.5);
    nodeHandlePtr_->param("sensor_pos_on_boom", sensorPosOnBoom_, 15.0);
    nodeHandlePtr_->param("circle_radius", circleRadius_, 5.0);
    nodeHandlePtr_->param("rotate_speed", rotateSpeed_, 10.0);
    nodeHandlePtr_->param("load_vert_axis_side_", loadVertAxisSide_, 1.5); // [m] vertically to ground plane
    nodeHandlePtr_->param("load_boompara_axis_side", loadBoomParaAxisSide_, 1.5); // [m] (parallel with boom)
    nodeHandlePtr_->param("load_boomperp_axis_side", loadBoomPerpAxisSide_, 4.0); // [m] (perpendicular to boom)
    nodeHandlePtr_->param("reference_frame", referenceFrame_, defaultReferenceFrame);
    nodeHandlePtr_->param("sensor_model_name", sensorModelFrameId_, defaultSensorModelFrameId);
    nodeHandlePtr_->param("rope1_link_model_frame_id", rope1LinkModelFrameId_, defaultRope1LinkModelFrameId);
    nodeHandlePtr_->param("warning_threshold", warningThreshold_, 10); // todo 3
    nodeHandlePtr_->param("will_pablishing_cloud", willPablishingCloud_, willPablishingCloud_);
    nodeHandlePtr_->param("warning_margin_vert", warningMarginVert_, 0.5); // [m]
    nodeHandlePtr_->param("warning_margin_boom_para", warningMarginBoomPara_, 2.0); // [m] (parallel with boom)
    nodeHandlePtr_->param("warning_margin_boom_perp", warningMarginBoomPerp_, 0.5); // [m] (perpendicular to boom)
    nodeHandlePtr_->param("length_correction", lengthCorrection_, 0.0); // [m] (perpendicular to boom)
    nodeHandlePtr_->param("rope_length", ropeLength_, 11.5 + 9.15);
    nodeHandlePtr->param("default_yaw_joint", defaultYawJoint_, -(0.0)); // -(def_yaw)
    nodeHandlePtr->param("default_pitch_joint", defaultPitchJoint_, -M_PI/4 + M_PI/2); // def_pitch + M_PI/2
    nodeHandlePtr->param("boom_link_height", boomLinkHeight_, 1.93);
    nodeHandlePtr->param("load_axis_viewer_factor", loadAxisViewerFactor_, 2.0);
    nodeHandlePtr->param("load_axis_viewer_num_of_points", loadAxisViewerNumOfPoints_, 31);
    nodeHandlePtr->param("warning_vis_model_name", warningVisModelName_, defaultWarningVisModelName);
    // nodeHandlePtr->param("warning_viz_model_x", warningVizModelX_, 20.0);
    // nodeHandlePtr->param("warning_viz_model_warned_y", warningVizModelWarnedY_, -10.0);
    // nodeHandlePtr->param("warning_viz_model_not_warned_y", warningVizModelNotWarnedY_, -80.0);
    // nodeHandlePtr->param("warning_viz_model_z", warningVizModelZ_, 0.0);
    nodeHandlePtr->param("enable_PCA", enablePCA_, true);
    // nodeHandlePtr->param("warning_viz_model_warned_z", warningVizModelWarnedZ_, 15.0);
    // nodeHandlePtr->param("warning_viz_model_not_warned_z", warningVizModelNotWarnedZ_, 85.0);
    nodeHandlePtr->param("correction_cw_x", correctionCWX_, 3.4);
    nodeHandlePtr->param("correction_cw_y", correctionCWY_, 2.0);
    nodeHandlePtr->param("correction_ccw_x", correctionCCWX_, 1.5);
    nodeHandlePtr->param("correction_ccw_y", correctionCCWY_, -0.2);
    nodeHandlePtr->param("is_offline", isOffLine_, true);
    nodeHandlePtr->param("sensor_cylinder_distance", sensorCylinderDistanceNotSim_, 3.0);
    nodeHandlePtr->param("sensor_height", sensorHeightNotSim_, 19.6);

    if (loadAxisViewerNumOfPoints_ % 2 == 0)
    {
        loadAxisViewerNumOfPoints_++;
    }

    // moveAnglInCycle_ = pcl::deg2rad(rotateSpeed_) * SUBSCRIBE_CYCLE;

    if (willPablishingCloud_)
    {
        ROS_INFO("will_pablishing_cloud was set true");
    } else
    {
        ROS_INFO("will_pablishing_cloud was set false");
    }

    if (isSimulation_)
    {
        moveModel(0.0, 0.0, 0.0);
    }
}

CollisionWarning::~CollisionWarning()
{
    delete syncSim_;
    delete syncNotSim_;
}

void CollisionWarning::callbackSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg,
        const sensor_msgs::JointState::ConstPtr& jointMsg,
        const sensor_msgs::JointState::ConstPtr& angleMsg)
{
    if (isFirst_)
    {
        previousTime_ = ros::Time::now();
        isFirst_ = false;
        return;
    }
    nowTime_ = ros::Time::now();
    ros::Duration callbackDuration = nowTime_ - previousTime_;
    previousTime_ = nowTime_;
    double subscribeCycle = callbackDuration.sec + (static_cast<double>(callbackDuration.nsec) * 0.000000001);

    std::vector<std::string> nameStrs = jointMsg->name;
    auto itrPitchJoint = std::find(nameStrs.begin(), nameStrs.end(), "pitch_joint");
    size_t indexPitchJoint = std::distance(nameStrs.begin(), itrPitchJoint);
    auto itrYawJoint = std::find(nameStrs.begin(), nameStrs.end(), "yaw_joint");
    size_t indexYawJoint = std::distance(nameStrs.begin(), itrYawJoint);
    if (nameStrs.size() == indexPitchJoint || nameStrs.size() == indexYawJoint)
    {
        ROS_ERROR_STREAM("Error at callback() in " << __FILE__ << ", __LINE__ = " << __LINE__);
        // return false;
    } else {
        moveAnglInCycle_ = pcl::deg2rad(rotateSpeed_) * subscribeCycle;
        // ROS_INFO_STREAM("callbackDuration.sec = " << callbackDuration.sec << " , callbackDuration.nsec" << callbackDuration.nsec);
        // ROS_INFO_STREAM("moveAnglInCycle_ = " << moveAnglInCycle_);

        // static tf2_ros::TransformBroadcaster br;
        // geometry_msgs::TransformStamped transformStamped;
        
        // transformStamped.header.stamp = ros::Time::now();
        // transformStamped.header.frame_id = (loopIdx == 0 ? "upper_link_processed" : "lower_link");
        // transformStamped.child_frame_id = (loopIdx == 0 ? "boom_link_processed" : "upper_link_processed");

        // transformStamped.transform.translation.x = (loopIdx == 0 ? boomLinkDefaultX_ : upperLinkDefaultX_);
        // transformStamped.transform.translation.y = (loopIdx == 0 ? boomLinkDefaultY_ : upperLinkDefaultY_);
        // transformStamped.transform.translation.z = (loopIdx == 0 ? boomLinkDefaultZ_ : upperLinkDefaultZ_);
        // tf2::Quaternion q;
        // q.setRPY(0.0,
        //             (loopIdx == 0 ? defaultPitchJoint_ - jointMsg->position[indexPitchJoint] : 0.0),
        //             (loopIdx == 0 ? 0.0 : defaultYawJoint_ - jointMsg->position[indexPitchJoint]));
        // transformStamped.transform.rotation.x = q.x();
        // transformStamped.transform.rotation.y = q.y();
        // transformStamped.transform.rotation.z = q.z();
        // transformStamped.transform.rotation.w = q.w();

        // br.sendTransform(transformStamped);
        // // ROS_INFO("broadcasted");

        // float urgStart, urgIncrement;
        // urgStart = scanMsg->angle_min;
        // urgIncrement = scanMsg->angle_increment;
        // for (size_t j = 0; j < scanMsg->ranges.size(); j+=(scanMsg->ranges.size()/4)) {
        //     // SICKのスキャン角度を更新
        //     float urgAngle = urgStart + urgIncrement * j;

        //     // 距離データと反射強度の値を取得
        //     float range = scanMsg->ranges[j];
        //     // float intensity = scanMsg->intensities[j];
        //     ROS_INFO_STREAM("urgAngle = " << urgAngle << " , range = " << range);
        //     // urgAngle = 0.00787349 , range = 12.6604 tumari,urgAngle=0 ha masita no toki 
        //     // urgAngle = -1.56686 , range = 21.1593 tumari, clock wise ga plus

        //     // 距離データから3次元位置を計算
        //     // pcl::PointXYZI point = range2Point(range, intensity, urgAngle, panAngle);

        //     // keyScan.push_back(point);
        // }

        double pitchJoint = defaultPitchJoint_ + jointMsg->position[indexPitchJoint];
        double yawJoint = -(defaultYawJoint_ + jointMsg->position[indexYawJoint]);
        // ROS_INFO_STREAM("p1 jointMsg->position[indexPitchJoint] = " << jointMsg->position[indexPitchJoint]
        //     << ", jointMsg->position[indexYawJoint] = " << jointMsg->position[indexYawJoint]);
        // ROS_INFO_STREAM("p1 pitchJoint = " << pitchJoint << ", yawJoint = " << yawJoint);

        double lengthToLoad = (boomLength_ - sensorPosOnBoom_ + lengthCorrection_) * std::cos(pitchJoint);
        double thetaAngle;
        if (circleRadius_ > lengthToLoad)
        {
            thetaAngle = std::asin(1.0); // = pi/2
        }
        else
        {
            thetaAngle = std::asin(circleRadius_ / lengthToLoad);
        }

        // if (warnOfCollision(scanMsg, thetaAngle, lengthToLoad, pitchJoint))
        // {
        //     ROS_WARN_STREAM("It warned of collision");
        // } else
        // {
        //     ROS_INFO_STREAM("It didn't warn of collision");
        // }

        bool resultCollisionWarning = false;
        if (laserScanData_.size() >= 2)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr generatedCloud(new pcl::PointCloud<pcl::PointXYZI>);
            // generatePointCloud(generatedCloud);
            // ROS_INFO_STREAM("laserScanData_.size() = " << laserScanData_.size()
            //     << ", rotatingBaseStateData_.size() = " << rotatingBaseStateData_.size()
            //     << ", translateZPositionData_.size() = " << translateZPositionData_.size()
            //     << ", rotateYawAngleData_.size() = " << rotateYawAngleData_.size()
            //     );
            ScanPlans2PointCloud::generatePointCloud(*generatedCloud,
                laserScanData_, rotatingBaseStateData_,
                0, laserScanData_.size() - 1,
                CANT_SCAN_DISTANCE, SCAN_DISTANCE_MAX, isSimulation_, (isSimulation_ ? SCAN_PLANE_DISTANCE_SIM : SCAN_PLANE_DISTANCE),
                sensorPoseStrs_);

            if (moveRotatingBase(thetaAngle))
            {
                resetVectors();
            }
            // double translateZPosition = boomLength_ * std::sin(pitchJoint);
            // double rotateYawAngle = yawJoint;
            // if (!isSimulation_)
            // {
            //     for (auto& it : *generatedCloud)
            //     {
            //         it.x = cos(rotateYawAngle) * it.x - sin(rotateYawAngle) * it.y;
            //         it.y = sin(rotateYawAngle) * it.x + cos(rotateYawAngle) * it.y; 
            //         it.z += translateZPosition;
            //     }
            // }

            // //save pcd
            // time_t now = time(NULL);
            // struct tm *pnow = localtime(&now);
            // std::string time_string
            //     = "_" + std::to_string(pnow->tm_year + 1900) + "-" + std::to_string(pnow->tm_mon + 1) + "-" + std::to_string(pnow->tm_mday)
            //     + "_" + std::to_string(pnow->tm_hour) + "-" + std::to_string(pnow->tm_min) + "-" + std::to_string(pnow->tm_sec);

            // std::string path = PACKAGE_PATH + "/scans/collisionWarniningTestPcd" + time_string + ".pcd";
            // pcl::io::savePCDFileBinary(path, generatedCloud);
            // // ROS_INFO_STREAM("pcd exported");

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if (isFirstScan_)
            {
                isFirstScan_ = false;
            }
            else if (warnOfCollision(generatedCloud, coloredCloud, pitchJoint, yawJoint))
            {
                ROS_WARN_STREAM("It warned of collision");
                resultCollisionWarning = true;
                // moveModel(warningVizModelX_, warningVizModelWarnedY_, warningVizModelZ_);
            } else
            {
                ROS_INFO_STREAM("It didn't warn of collision");
                resultCollisionWarning = false;
            }

            if (willPablishingCloud_)
            {
                sensor_msgs::PointCloud2 result;
                pcl::toROSMsg(*coloredCloud, result);

                // ある地点における3次元スキャンデータをPointCloud2メッセージでpublish
                result.header.frame_id = "laser";
                result.header.stamp = ros::Time::now();
                liveScanPub_.publish(result);
            }
        }

        sensor_msgs::JointState rotatingBaseMsg;
        rotatingBaseMsg.position.resize(1);
        rotatingBaseMsg.position[0] = nowAngleSim_;
        // rotatingBaseMsg.position[0] = angleMsg->position[0];
        // ROS_INFO_STREAM("nowAngleSim_ = " << nowAngleSim_);
        laserScanData_.push_back(*scanMsg);
        rotatingBaseStateData_.push_back(rotatingBaseMsg);
        SensorPoseStr sensorPoseStr(
            sensorPosOnBoom_ * std::cos(pitchJoint) * std::sin(yawJoint),
            sensorPosOnBoom_ * std::cos(pitchJoint) * std::cos(yawJoint),
            sensorPosOnBoom_ * std::sin(pitchJoint),
            yawJoint);
        sensorPoseStrs_.push_back(sensorPoseStr);
        // translateZPositionData_.push_back(sensorPosOnBoom_ * std::sin(pitchJoint));
        // rotateYawAngleData_.push_back(yawJoint);
        // ROS_INFO_STREAM("translateZPosition = " << sensorPosOnBoom_ * std::sin(pitchJoint) << ", rotateYawAngle = " << yawJoint);
        // ROS_INFO_STREAM("pitchJoint = " << pitchJoint << ", yawJoint = " << yawJoint);

        // if (isClockWise_) // --
        // {
        //     moveRotatingBase(-thetaAngle);
        // }
        // else // ++
        // {
        //     moveRotatingBase(thetaAngle);
        // }

        std_msgs::Bool warningResultBool;
        warningResultBool.data = resultCollisionWarning;
        collisionWarningResultPub_.publish(warningResultBool);
    }
}

void CollisionWarning::executeWarningSystem()
{
    std::thread scanThread(&CollisionWarning::receiveScanDataThread, this);

    // if (!movePTU())  return false;
    // double lengthToLoad = (boomLength_ - sensorPosOnBoom_ + lengthCorrection_) * std::cos(pitchJoint);
    double lengthToLoad = sensorCylinderDistanceNotSim_;
    double thetaAngle;
    if (circleRadius_ > lengthToLoad)
    {
        thetaAngle = pcl::rad2deg(std::asin(1.0)); // = pi/2
    }
    else
    {
        thetaAngle = pcl::rad2deg(std::asin(circleRadius_ / lengthToLoad));
    }

    // rotating baseを動かす処理
    bool isCWLocal = true;
    while (ros::ok())
    {
        orion_rotating_base::sendCommand cmd;
        cmd.request.end_pos        = (isCWLocal ? -thetaAngle : thetaAngle);
        cmd.request.speed          = static_cast<int>(rotateSpeed_);
        // cmd.request.end_pos        = 90.0;
        // cmd.request.speed          = 10;

        // 以下は実際にサービスで回転台を動作させている
        // mutex_.lock();
        // isRotatingBaseWork_ = true;
        // mutex_.unlock();

        if (rotatingBaseActionClient_.call(cmd)) {
            // mutex_.lock();
            // isRotatingBaseWork_ = false;
            // mutex_.unlock();

            // ROS_INFO("Service call successful.");
            // return true;
        }
        else {
            // mutex_.lock();
            // isRotatingBaseWork_ = false;
            // mutex_.unlock();

            ROS_ERROR("Failed to call service! at CollisionWarning::executeWarningSystem()");
            // return false;
        }

        isCWLocal = !isCWLocal;
        resetVectors();
    }

    scanThread.join();
}

void CollisionWarning::receiveScanDataThread()
{
    // mutex_.lock(); // ここでスレッド間による共通の変数をロック

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CollisionWarning::callbackNotSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg, const sensor_msgs::JointState::ConstPtr& angleMsg)
{
    // bool resultCollisionWarning = false;
    if (laserScanData_.size() >= 2)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr generatedCloud(new pcl::PointCloud<pcl::PointXYZI>);
        // generatePointCloud(generatedCloud);
        // ROS_INFO_STREAM("laserScanData_.size() = " << laserScanData_.size()
        //     << ", rotatingBaseStateData_.size() = " << rotatingBaseStateData_.size()
        //     << ", translateZPositionData_.size() = " << translateZPositionData_.size()
        //     << ", rotateYawAngleData_.size() = " << rotateYawAngleData_.size()
        //     );
        SensorPoseStr zeroPoseStr(0.0,0.0,0.0,0.0);
        std::vector<SensorPoseStr> sensorPoseStrs{zeroPoseStr};
        ScanPlans2PointCloud::generatePointCloud(*generatedCloud,
            laserScanData_, rotatingBaseStateData_,
            0, laserScanData_.size() - 1,
            CANT_SCAN_DISTANCE, SCAN_DISTANCE_MAX, isSimulation_, (isSimulation_ ? SCAN_PLANE_DISTANCE_SIM : SCAN_PLANE_DISTANCE),
            sensorPoseStrs);

        // if (moveRotatingBase(thetaAngle))
        // {
        //     resetVectors();
        // }
        // double translateZPosition = boomLength_ * std::sin(pitchJoint);
        // double rotateYawAngle = yawJoint;
        // if (!isSimulation_)
        // {
        //     for (auto& it : *generatedCloud)
        //     {
        //         it.x = cos(rotateYawAngle) * it.x - sin(rotateYawAngle) * it.y;
        //         it.y = sin(rotateYawAngle) * it.x + cos(rotateYawAngle) * it.y;
        //         it.z += translateZPosition;
        //     }
        // }

        // //save pcd
        // time_t now = time(NULL);
        // struct tm *pnow = localtime(&now);
        // std::string time_string
        //     = "_" + std::to_string(pnow->tm_year + 1900) + "-" + std::to_string(pnow->tm_mon + 1) + "-" + std::to_string(pnow->tm_mday)
        //     + "_" + std::to_string(pnow->tm_hour) + "-" + std::to_string(pnow->tm_min) + "-" + std::to_string(pnow->tm_sec);

        // std::string path = PACKAGE_PATH + "/scans/collisionWarniningTestPcd" + time_string + ".pcd";
        // pcl::io::savePCDFileBinary(path, generatedCloud);
        // // ROS_INFO_STREAM("pcd exported");

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // if (isFirstScan_)
        // {
        //     isFirstScan_ = false;
        // }
        if (warnOfCollision(generatedCloud, coloredCloud, 0.0, 0.0))
        {
            ROS_WARN_STREAM("It warned of collision");
            // resultCollisionWarning = true;
            // moveModel(warningVizModelX_, warningVizModelWarnedY_, warningVizModelZ_);
        } else
        {
            ROS_INFO_STREAM("It didn't warn of collision");
            // resultCollisionWarning = false;
        }

        if (willPablishingCloud_)
        {
            sensor_msgs::PointCloud2 result;
            pcl::toROSMsg(*coloredCloud, result);

            // ある地点における3次元スキャンデータをPointCloud2メッセージでpublish
            result.header.frame_id = "laser";
            result.header.stamp = ros::Time::now();
            liveScanPub_.publish(result);
        }
    }

    // sensor_msgs::JointState rotatingBaseMsg;
    // rotatingBaseMsg.position.resize(1);
    // rotatingBaseMsg.position[0] = nowAngleSim_;
    // // rotatingBaseMsg.position[0] = angleMsg->position[0];
    // // ROS_INFO_STREAM("nowAngleSim_ = " << nowAngleSim_);
    // laserScanData_.push_back(*scanMsg);
    // rotatingBaseStateData_.push_back(rotatingBaseMsg);
    // SensorPoseStr sensorPoseStr(
    //     sensorPosOnBoom_ * std::cos(pitchJoint) * std::sin(yawJoint),
    //     sensorPosOnBoom_ * std::cos(pitchJoint) * std::cos(yawJoint),
    //     sensorPosOnBoom_ * std::sin(pitchJoint),
    //     yawJoint);
    // sensorPoseStrs_.push_back(sensorPoseStr);
    // // translateZPositionData_.push_back(sensorPosOnBoom_ * std::sin(pitchJoint));
    // // rotateYawAngleData_.push_back(yawJoint);
    // // ROS_INFO_STREAM("translateZPosition = " << sensorPosOnBoom_ * std::sin(pitchJoint) << ", rotateYawAngle = " << yawJoint);
    // // ROS_INFO_STREAM("pitchJoint = " << pitchJoint << ", yawJoint = " << yawJoint);

    // if (isClockWise_) // --
    // {
    //     moveRotatingBase(-thetaAngle);
    // }
    // else // ++
    // {
    //     moveRotatingBase(thetaAngle);
    // }

    // std_msgs::Bool warningResultBool;
    // warningResultBool.data = resultCollisionWarning;
    // collisionWarningResultPub_.publish(warningResultBool);
}

bool CollisionWarning::warnOfCollision(pcl::PointCloud<pcl::PointXYZI>::Ptr& generatedCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& coloredCloud,
        double pitchJoint, double yawJoint)
{
    // static tf2_ros::Buffer tfBuffer;
    // static tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped nowTransformStamped;
    // // static size_t countWarn = 0;
    // try{
    //     nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, ros::Time(0));
    // }
    // catch (tf2::TransformException &ex) {
    //     // if (countWarn < 300)
    //     // {
    //     //     countWarn++;
    //         ROS_INFO("in warnOfCollision(), %s",ex.what());
    //     // }
    //     // ROS_WARN("in warnOfCollision(), %s",ex.what());
    //     return false;
    // }
    // double sensorCenterHeight = nowTransformStamped.transform.translation.z;

    // try{
    //     nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, rope1LinkModelFrameId_, ros::Time(0));
    // }
    // catch (tf2::TransformException &ex) {
    //     // if (countWarn < 300)
    //     // {
    //     //     countWarn++;
    //         ROS_INFO("in warnOfCollision(), %s",ex.what());
    //     // }
    //     // ROS_WARN("in warnOfCollision(), %s",ex.what());
    //     return false;
    // }
    // double rope1Height = nowTransformStamped.transform.translation.z;
    // double loadLowerHeight = rope1Height/2 - ropeLength_ - loadVertAxisSide_;

    double cylinderCenterX = 0.0;
    double cylinderCenterY = 0.0;
    // double cylinderLower = std::max(GROUND_PLANE_Z_VALUE + 0.1, loadLowerHeight) - sensorCenterHeight;
    double cylinderLower = 0.0;
    // ROS_INFO_STREAM("rope1Height = " << rope1Height << ", ropeLength_ = " << ropeLength_ << ", loadVertAxisSide_ = " << loadVertAxisSide_);
    // ROS_INFO_STREAM("cylinderLower = " << cylinderLower << ", sensorCenterHeight = " << sensorCenterHeight);
    double cylinderUpper = 0.0;
    if (isSimulation_)
    {
        double cylinderCenterX = boomLength_ * std::cos(pitchJoint) * std::sin(yawJoint);
        double cylinderCenterY = boomLength_ * std::cos(pitchJoint) * (-std::cos(yawJoint));
        // double cylinderLower = std::max(GROUND_PLANE_Z_VALUE + 0.1, loadLowerHeight) - sensorCenterHeight;
        double cylinderLower = std::max(GROUND_PLANE_Z_VALUE + 0.1, boomLength_ * std::sin(pitchJoint) - ropeLength_ - loadVertAxisSide_/2) - boomLinkHeight_;
        // ROS_INFO_STREAM("rope1Height = " << rope1Height << ", ropeLength_ = " << ropeLength_ << ", loadVertAxisSide_ = " << loadVertAxisSide_);
        // ROS_INFO_STREAM("cylinderLower = " << cylinderLower << ", sensorCenterHeight = " << sensorCenterHeight);
        double cylinderUpper = (boomLength_ - (circleRadius_/std::cos(pitchJoint))) * std::sin(pitchJoint) - warningMarginVert_;
    } else
    {
        double cylinderCenterX = 0.0;
        double cylinderCenterY = -sensorCylinderDistanceNotSim_;
        double cylinderLower = -sensorHeightNotSim_;
        double cylinderUpper = 0.0;
    }

    pcl::copyPointCloud(*generatedCloud, *coloredCloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCA(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool wasSucceededPCA = false;
    std::vector<double> pcaedCenterValue(3);
    std::vector<double> firstPrincipalAxis(3);
    if (enablePCA_)
    {
        pcl::copyPointCloud(*coloredCloud, *cloudPCA);

            // // detect surface
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::copyPointCloud(*generatedCloud, *cloud);
            // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            // pcl::SACSegmentation<pcl::PointXYZI> seg;
            // seg.setOptimizeCoefficients(true); //If it doesn't use coefficients,this is able to be false.
            // seg.setModelType(pcl::SACMODEL_PLANE);
            // seg.setMethodType(pcl::SAC_RANSAC);
            // seg.setMaxIterations(100);
            // seg.setDistanceThreshold(0.14);
            // seg.setInputCloud(cloud);
            // seg.segment(*inliers, *coefficients);

            // pcl::ExtractIndices<pcl::PointXYZI> extract;
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZI>);//点群を保存
            // extract.setInputCloud(cloud);
            // extract.setIndices(inliers);
            // extract.setNegative(true);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
            // extract.filter(*cloud_output);

        // pick points around load
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        // double pickingLengthX;
        // double pickingLengthY;
        // if (std::abs(yawJoint) < M_PI_4)
        // {
        //     pickingLengthX = loadBoomPerpAxisSide_ +
        //         ((std::sqrt(std::pow(loadBoomPerpAxisSide_, 2.0) + std::pow(loadBoomParaAxisSide_, 2.0)) - loadBoomPerpAxisSide_) * (std::abs(yawJoint) / M_PI_4));
        //     pickingLengthY = loadBoomParaAxisSide_ +
        //         ((std::sqrt(std::pow(loadBoomPerpAxisSide_, 2.0) + std::pow(loadBoomParaAxisSide_, 2.0)) - loadBoomParaAxisSide_) * (std::abs(yawJoint) / M_PI_4));
        // } else
        // {
        //     pickingLengthX = loadBoomParaAxisSide_ +
        //         ((std::sqrt(std::pow(loadBoomPerpAxisSide_, 2.0) + std::pow(loadBoomParaAxisSide_, 2.0)) - loadBoomParaAxisSide_) *
        //          (M_PI_2 - (std::abs(yawJoint)) / M_PI_4));
        //     pickingLengthY = loadBoomPerpAxisSide_ +
        //         ((std::sqrt(std::pow(loadBoomPerpAxisSide_, 2.0) + std::pow(loadBoomParaAxisSide_, 2.0)) - loadBoomPerpAxisSide_) *
        //          (M_PI_2 - (std::abs(yawJoint)) / M_PI_4));
        // }

        bool isYawJointZero = true;

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        if (isYawJointZero)
        {
            pass.setInputCloud (cloudPCA);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-loadBoomPerpAxisSide_*2.0, loadBoomPerpAxisSide_*2.0);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloudPCA);
            // printf("%f\n",cloud->points.size());
            pass.setInputCloud (cloudPCA);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(-(boomLength_ * std::cos(pitchJoint) + loadBoomParaAxisSide_*1.0),
                                -(boomLength_ * std::cos(pitchJoint) - loadBoomParaAxisSide_*2.8));
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloudPCA);
            // for (size_t i = 0; i < cloudPCA->size(); i++) {
            //     cloudPCA->points[i].y += boomLength_ * std::cos(pitchJoint) - loadBoomParaAxisSide_*1.7;
            // }
        } else
        {
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud (cloudPCA);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(boomLength_ * std::cos(pitchJoint) - loadBoomParaAxisSide_*2.8,
                                boomLength_ * std::cos(pitchJoint) + loadBoomParaAxisSide_*1.0);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloudPCA);
            // printf("%f\n",cloud->points.size());
            pass.setInputCloud (cloudPCA);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(-loadBoomPerpAxisSide_*0.8, loadBoomPerpAxisSide_*0.8);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloudPCA);
            // for (size_t i = 0; i < cloudPCA->size(); i++) {
            //     cloudPCA->points[i].x -= boomLength_ * std::cos(pitchJoint) - loadBoomParaAxisSide_*1.7;
            // }
        }

        pass.setInputCloud (cloudPCA);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(std::max(GROUND_PLANE_Z_VALUE + 0.1, cylinderLower - loadVertAxisSide_), cylinderLower + loadVertAxisSide_ * 1.5);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloudPCA);
        // return cloud_filtered;
        // for (size_t i = 0; i < cloudPCA->size(); i++) {
        //     cloudPCA->points[i].z += loadVertAxisSide_ * 0.5;
        // }

        if (cloudPCA->size() > 0)
        {
            std::vector<double> pcaedMinValue(3, DBL_MAX);
            std::vector<double> pcaedMaxValue(3, -DBL_MAX);
            for (size_t loopInx = 0; loopInx < 3; loopInx++)
            {
                for (size_t loopJnx = 0; loopJnx < cloudPCA->size(); loopJnx++)
                {
                    double thisAxisValue = (loopInx == 0 ? cloudPCA->points[loopJnx].x :
                                            (loopInx == 1 ? cloudPCA->points[loopJnx].y : cloudPCA->points[loopJnx].z));
                    if (thisAxisValue < pcaedMinValue[loopInx])
                    {
                        pcaedMinValue[loopInx] = thisAxisValue;
                    }
                    if (thisAxisValue > pcaedMaxValue[loopInx])
                    {
                        pcaedMaxValue[loopInx] = thisAxisValue;
                    }
                }
            }
            for (size_t loopInx = 0; loopInx < 3; loopInx++)
            {
                pcaedCenterValue[loopInx] = (pcaedMinValue[loopInx] + pcaedMaxValue[loopInx]) / 2;
            }
        }

        pcl::PCA<pcl::PointXYZRGB> pca;
        // Eigen::Vector3f& eigen_values;
        // Eigen::Matrix3f& eigen_vectors;
        try{
            pca.setInputCloud(cloudPCA);
            // std::vector<float> dist;
            // pcl::PointIndices::Ptr pca_indices(new pcl::PointIndices());
            // const int size = indices_->size();

            // Eigen::MatrixXf& coefficients = pca.getCoefficients();
            // Eigen::Vector3f& eigen_values = pca.getEigenValues();
            Eigen::Matrix3f& eigen_vectors = pca.getEigenVectors();
            // Eigen::Vector4f& mean = pca.getMean();

            // PointXYZD& dst = output.points[i];
            // dst.lambda[0] = eigen_values(0);
            // dst.lambda[1] = eigen_values(1);
            // dst.lambda[2] = eigen_values(2);

            // ROS_INFO_STREAM("coefficients = \n" << coefficients);
            // ROS_INFO_STREAM("eigen_values(0) = " << eigen_values(0) << " ,eigen_values(1) = " << eigen_values(1) << " ,eigen_values(2) = " << eigen_values(2));
            // ROS_INFO_STREAM("eigen_vectors = \n" << eigen_vectors);
            // ROS_INFO_STREAM("mean(0) = " << mean(0) << " ,mean(1) = " << mean(1) << " ,mean(2) = " << mean(2) << " ,mean(3) = " << mean(3));

            // boost::shared_ptr <std::vector<int> > indices_ : 指定された点群input_のうち特徴量を計算する対象のインデックス
            // for ( int i = 0; i < size; i++ ) {
                // int index = (*indices_)[i];
                // dist.clear();
                // pca_indices->indices.clear();
                // // 親クラスのメンバ関数 searchForNeighbors で近傍点を探す
                // this->searchForNeighbors(index, search_parameter_, pca_indices->indices, dist);
                // // 見つかった近傍点のインデックスを指定して主成分分析
                // pca.setIndices(pca_indices);
                // Eigen::Vector3f& eigen_values = pca.getEigenValues();
                // PointXYZD& dst = output.points[i];
                // dst.lambda[0] = eigen_values(0);
                // dst.lambda[1] = eigen_values(1);
                // dst.lambda[2] = eigen_values(2);
                // // 固有値＝各主成分方向の分散　を比較して次元特徴を計算する
                // dst.dimension = (uint8_t)get_dimension_feature(dst.lambda);
                // const pcl::PointXYZRGB& p = input_->points[index];
                // dst.x = p.x;
                // dst.y = p.y;
                // dst.z = p.z;
            // }
            // output.is_dense = true;
            // output.height = 1;
            // output.width = output.points.size();
            // wasSucceededPCA = true;
            for (size_t loopInx = 0; loopInx < firstPrincipalAxis.size(); loopInx++)
            {
                firstPrincipalAxis[loopInx] = eigen_vectors(loopInx, 0);
            }
            ROS_INFO_STREAM("center x,y,z = " << pcaedCenterValue[0] << ", " << pcaedCenterValue[1] << ", " << pcaedCenterValue[2]);
            ROS_INFO_STREAM("firstPrincipalAxis x,y,z = " << firstPrincipalAxis[0] << ", " << firstPrincipalAxis[1] << ", " << firstPrincipalAxis[2]);
            // ROS_INFO_STREAM("cloudPCA->width = " << cloudPCA->width << ", cloudPCA->height" << cloudPCA->height);
            // cloudPCA->points.resize(cloudPCA->size() + 1)
            // cloudPCA->points[cloudPCA->size() - 1].x = pcaedCenterValue[0] + firstPrincipalAxis[0];
            // cloudPCA->points[cloudPCA->size() - 1].y = pcaedCenterValue[1] + firstPrincipalAxis[1];
            // cloudPCA->points[cloudPCA->size() - 1].z = pcaedCenterValue[2] + firstPrincipalAxis[2];
            // cloudPCA->points[cloudPCA->size() - 1].r = 0;
            // cloudPCA->points[cloudPCA->size() - 1].g = 255;
            // cloudPCA->points[cloudPCA->size() - 1].b = 0;

            // for (int loopInx = 0; loopInx < loadAxisViewerNumOfPoints_; loopInx++)
            // {
            //     double localFactor =
            //         static_cast<double>(loopInx - (loadAxisViewerNumOfPoints_ / 2)) / static_cast<double>(loadAxisViewerNumOfPoints_ / 2)
            //         * loadAxisViewerFactor_;
            //     pcl::PointXYZRGB resultPoint;
            //     resultPoint.x = pcaedCenterValue[0] + (firstPrincipalAxis[0] * localFactor);
            //     resultPoint.y = pcaedCenterValue[1] + (firstPrincipalAxis[1] * localFactor);
            //     resultPoint.z = pcaedCenterValue[2] + (firstPrincipalAxis[2] * localFactor);
            //     resultPoint.r = 0;
            //     resultPoint.g = 255;
            //     resultPoint.b = 0;
            //     cloudPCA->width++;
            //     cloudPCA->points.push_back(resultPoint);
            // }

            // pcl::PointXYZRGB resultPoint;
            // resultPoint.x = pcaedCenterValue[0] + firstPrincipalAxis[0];
            // resultPoint.y = pcaedCenterValue[1] + firstPrincipalAxis[1];
            // resultPoint.z = pcaedCenterValue[2] + firstPrincipalAxis[2];
            // resultPoint.r = 0;
            // resultPoint.g = 255;
            // resultPoint.b = 0;
            // cloudPCA->width++;
            // cloudPCA->points.push_back(resultPoint);
            // pcl::PointXYZRGB resultPoin1;
            // resultPoin1.x = pcaedCenterValue[0];
            // resultPoin1.y = pcaedCenterValue[1];
            // resultPoin1.z = pcaedCenterValue[2];
            // resultPoin1.r = 0;
            // resultPoin1.g = 255;
            // resultPoin1.b = 0;
            // cloudPCA->width++;
            // cloudPCA->points.push_back(resultPoin1);
            // pcl::PointXYZRGB resultPoin2;
            // resultPoin2.x = pcaedCenterValue[0] - firstPrincipalAxis[0];
            // resultPoin2.y = pcaedCenterValue[1] - firstPrincipalAxis[1];
            // resultPoin2.z = pcaedCenterValue[2] - firstPrincipalAxis[2];
            // resultPoin2.r = 0;
            // resultPoin2.g = 255;
            // resultPoin2.b = 0;
            // cloudPCA->width++;
            // cloudPCA->points.push_back(resultPoin2);
            wasSucceededPCA = true;
        }
        catch (pcl::InitFailedException &ex) {
            wasSucceededPCA = false;
            // ROS_WARN("%s",ex.what());
            // return false;
        }
    }

    size_t collisionPointsCount = 0;
    bool warnFlag = false;
    std::vector<double> warnedPoint(3, 0.0);
    for (size_t i = 0; i < coloredCloud->size(); i++) {
        double point_x = coloredCloud->points[i].x;
        double point_y = coloredCloud->points[i].y;
        double point_z = coloredCloud->points[i].z;

        if ((std::pow(point_x - cylinderCenterX, 2.0) + std::pow(point_y - cylinderCenterY, 2.0) < std::pow(circleRadius_, 2.0))
            && (point_z > GROUND_PLANE_Z_VALUE + 0.33) && (point_z < cylinderUpper))
        {       // if point in cylindrical area
            // if ((point_x < -loadBoomPerpAxisSide_ / 2 - warningMarginBoomPerp_) || (point_x > loadBoomPerpAxisSide_ / 2 + warningMarginBoomPerp_)
            //     || (point_y > -lengthToLoad + loadBoomParaAxisSide_ / 2 + warningMarginBoomPara_))
            if (!isSimulation_
             || (isSimulation_ && ((std::pow(point_x, 2.0) + std::pow(point_y, 2.0) < std::pow(boomLength_ * std::cos(pitchJoint) - loadBoomParaAxisSide_*1.6, 2.0))
                || (std::pow(point_x, 2.0) + std::pow(point_y, 2.0) > std::pow(boomLength_ * std::cos(pitchJoint) + loadBoomParaAxisSide_*1.3, 2.0)))))
                // || (std::pow(point_x, 2.0) + std::pow(point_y, 2.0) > std::pow(boomLength_ * std::cos(pitchJoint) + loadBoomParaAxisSide_*0.2, 2.0))) todo
            {       // if point is not part of the load
                // ROS_INFO_STREAM("-loadBoomPerpAxisSide_ / 2 - warningMarginBoomPerp_ = " << -loadBoomPerpAxisSide_ / 2 - warningMarginBoomPerp_
                //     << ", loadBoomPerpAxisSide_ / 2 + warningMarginBoomPerp_ = " << loadBoomPerpAxisSide_ / 2 + warningMarginBoomPerp_
                //     << ", -lengthToLoad + loadBoomParaAxisSide_ / 2 + warningMarginBoomPara_ = " << -lengthToLoad + loadBoomParaAxisSide_ / 2 + warningMarginBoomPara_);
                // ROS_INFO_STREAM("point_x = " << point_x << ", point_y = " << point_y << ", point_z = " << point_z);
                collisionPointsCount++;
                coloredCloud->points[i].r = 255;
                coloredCloud->points[i].g = 0;
                coloredCloud->points[i].b = 0;
                if (collisionPointsCount >= static_cast<size_t>(warningThreshold_))
                {
                    // return true; // warn of collision
                    warnFlag = true;
                    warnedPoint[0] = point_x;
                    warnedPoint[1] = point_y;
                    warnedPoint[2] = point_z;
                }
            } else
            {       // if point is part of the load
                coloredCloud->points[i].r = 0;
                coloredCloud->points[i].g = 0;
                coloredCloud->points[i].b = 255;
            }
        } else
        {
            coloredCloud->points[i].r = 255;
            coloredCloud->points[i].g = 255;
            coloredCloud->points[i].b = 255;
        }
    }

    if (isSimulation_)
    {
        if ((laserScanData_.size() >= 2 && isClockWise_) || (laserScanData_.size() < 2 && !isClockWise_))
        {
            moveModel(-warnedPoint[1] + correctionCWX_, warnedPoint[0] + correctionCWY_, 0.0);
        }
        else
        {
            moveModel(-warnedPoint[1] + correctionCCWX_, warnedPoint[0] + correctionCCWY_, 0.0);
        }
    }

    // pcl::copyPointCloud(*cloudPCA, *coloredCloud);
    if (enablePCA_ && wasSucceededPCA)
    {
        for (int loopInx = 0; loopInx < loadAxisViewerNumOfPoints_; loopInx++)
        {
            double localFactor =
                static_cast<double>(loopInx - (loadAxisViewerNumOfPoints_ / 2)) / static_cast<double>(loadAxisViewerNumOfPoints_ / 2)
                * loadAxisViewerFactor_;
            pcl::PointXYZRGB resultPoint;
            resultPoint.x = pcaedCenterValue[0] + (firstPrincipalAxis[0] * localFactor);
            resultPoint.y = pcaedCenterValue[1] + (firstPrincipalAxis[1] * localFactor);
            resultPoint.z = pcaedCenterValue[2] + (firstPrincipalAxis[2] * localFactor);
            resultPoint.r = 0;
            resultPoint.g = 255;
            resultPoint.b = 0;
            coloredCloud->width++;
            coloredCloud->points.push_back(resultPoint);
        }
    }
    // *coloredCloud += *cloudPCA;

    if (warnFlag)
    {
        return true; // warn of collision
    }

    // double distance_tmp;

    // distance_tmp = std::sqrt(
    //     std::pow(point_x, 2.0)
    //   + std::pow(point_y, 2.0)
    //   + std::pow(point_z, 2.0));

    // if (max_dist < distance_tmp)
    // {
    //   max_dist = distance_tmp;
    // }
    
    // if (std::isnan(point_x))
    // {
    //   cloudsize_without_nan--;
    // }
    
    return false;
}

void CollisionWarning::resetVectors()
{
    laserScanData_.clear();
    rotatingBaseStateData_.clear();
    sensorPoseStrs_.clear();
    // translateZPositionData_.clear();
    // rotateYawAngleData_.clear();
}

// void CollisionWarning::generatePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& keyScan)
// {
//     // スキャンデータから変換されたある地点における3次元データを保持
//     // pcl::PointCloud<pcl::PointXYZI>::Ptr keyScan;
  
//     // SICKから得られたスキャンデータを割り振る処理
//     // for (size_t i = scanStart; i <= scanEnd; ++i) {
//     for (size_t i = 0; i < laserScanData_.size(); ++i) {
//         sensor_msgs::LaserScan laserScan = laserScanData_[i];
//         sensor_msgs::JointState rotatingBaseState = rotatingBaseStateData_[i];
    
//         // スキャンデータが取得された際のパン角を更新
//         double panAngle = rotatingBaseState.position[0];
 
//         // scanRange_以外の範囲で取得したスキャンデータは変換に考慮しない
//         // if (panAngle > (isSimulation_ ? pcl::deg2rad(scanRange_) : scanRange_) || panAngle < (isSimulation_ ? pcl::deg2rad(-1.0 * scanRange_) : -1.0 * scanRange_))
//         // if (panAngle > pcl::deg2rad(scanRange_) || panAngle < pcl::deg2rad(-1.0 * scanRange_))
//         //     continue;
    
//         // SICKのスキャン開始位置[rad]および角度分解能[rad]の実測値を保持
//         float urgStart, urgIncrement;
//         urgStart = laserScan.angle_min;
//         urgIncrement = laserScan.angle_increment;
    
//         for (size_t j = 0; j < laserScan.ranges.size(); ++j) {
//             // SICKのスキャン角度を更新
//             float urgAngle = urgStart + urgIncrement * j;

//             // 距離データと反射強度の値を取得
//             float range = laserScan.ranges[j];
//             float intensity = laserScan.intensities[j];
      
//             // 距離データから3次元位置を計算
//             pcl::PointXYZI point = range2Point(range, intensity, urgAngle, panAngle);

//             keyScan.push_back(point);
//         }
//     }

//     // if (!isSimulation_)
//     // {
//     //      for (auto& it : keyScan)
//     //     {
//     //         it.y = cos(SEMICIRCLE_RAD) * it.y - sin(SEMICIRCLE_RAD) * it.z;
//     //         it.z = sin(SEMICIRCLE_RAD) * it.y + cos(SEMICIRCLE_RAD) * it.z; 
//     //     }
//     // }

//     // // リフレクタンス値の正規化
//     // normalizeReflectance(keyScan, refMin_, refMax_);

//     resetVectors();
// }

// void CollisionWarning::normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const double refMin, const double refMax)
// {
//     for (size_t idx = 0; idx < cloud.size(); ++idx) {
//         float intensity = cloud[idx].intensity;

//         // Scale intensity values from 0 to 1
//         if (intensity < refMin)       intensity = 0.0;
//         else if (intensity > refMax)  intensity = 1.0;
//         else                          intensity = (intensity - refMin) / (refMax - refMin);
    
//         cloud[idx].intensity = intensity;
//     }
// }

// pcl::PointXYZI CollisionWarning::range2Point(const float range, const float intensity, const float urgAngle, const double panAngle)
// {
//     pcl::PointXYZI point;
//     // double panAngRad = (isSimulation_ ? panAngle : pcl::deg2rad(panAngle));
//     double panAngRad = panAngle;

//     // 距離データの値が範囲外である場合
//     if (range <= CANT_SCAN_DISTANCE || range > SCAN_DISTANCE_MAX) {
//         // 計測することができなかったデータとして全ての値をNANとする
//         point.x = NAN;
//         point.y = NAN;
//         point.z = NAN;
//         point.intensity = NAN;

//         return point;
//     }

//     // double xForUv = (isSimulation_ ? SCAN_PLANE_DISTANCE_SIM : range * cos(urgAngle));
//     // double yForUv = (isSimulation_ ? range * sin(urgAngle) : SCAN_PLANE_DISTANCE);
//     // double zForUv = (isSimulation_ ? -range * cos(urgAngle) : -range * sin(urgAngle));
//     double xForUv = SCAN_PLANE_DISTANCE_SIM;
//     double yForUv = range * sin(urgAngle);
//     double zForUv = -range * cos(urgAngle);
//     Eigen::Vector3d uv0(xForUv, yForUv, zForUv);
//     Eigen::AngleAxisd rotation = Eigen::AngleAxisd(panAngRad, Eigen::Vector3d::UnitZ());
//     // ROS_INFO_STREAM("panAngRad = " << panAngRad);

//     // if(enableTiltCompensation_) {
//     //     double p = (panAngRad - calibPan0_) / (calibPan1_ - calibPan0_);
//     //     double tilt = (1 - p) * calibTilt1_;
//     //     rotation = Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()) * rotation;
//     // }

//     Eigen::Vector3d xyz = rotation * uv0;

//     point.x = xyz.x();
//     point.y = xyz.y();
//     point.z = xyz.z();
//     point.intensity = intensity;

//     return point;
// }

// bool CollisionWarning::warnOfCollision(const sensor_msgs::LaserScan::ConstPtr& scanMsg, double thetaAngle, double lengthToLoad, double boomPitch)
// {
//     double thetaAngl1 = std::atan2(loadBoomPerpAxisSide_ / 2, lengthToLoad - (loadBoomParaAxisSide_ / 2));
//     double thetaAngl2 = thetaAngle;

//     if ((nowAngleSim_ < 0 && (nowAngleSim_ >= -thetaAngl1 || nowAngleSim_ < -thetaAngl2))
//     || (nowAngleSim_ >= 0 && (nowAngleSim_ <= thetaAngl1 || nowAngleSim_ > thetaAngl2)))
//     {
//         return false;
//     }

//     static tf2_ros::Buffer tfBuffer;
//     static tf2_ros::TransformListener tfListener(tfBuffer);
//     geometry_msgs::TransformStamped nowTransformStamped;
//     try{
//         nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, ros::Time(0));
//     }
//     catch (tf2::TransformException &ex) {
//         ROS_WARN("%s",ex.what());
//         return false;
//     }
//     double sensorCenterHeight = nowTransformStamped.transform.translation.z;
    
//     double phiAngl1 = atan2(sensorCenterHeight, lengthToLoad - circleRadius_);
//     double phiAngl2 = atan2(sensorCenterHeight, lengthToLoad + circleRadius_);
//     double phiAngl3 = boomPitch;

//     float urgStart, urgIncrement;
//     urgStart = scanMsg->angle_min;
//     urgIncrement = scanMsg->angle_increment;
//     for (size_t j = 0; j < scanMsg->ranges.size(); j++) {
//         float urgAngle = urgStart + urgIncrement * j;
//         float nowPhiAngle = -std::asin(1.0) - urgAngle;
//         float laserDistance = scanMsg->ranges[j];
//         float distanceMin = (lengthToLoad - circleRadius_) / std::cos(nowPhiAngle);

//         if (nowPhiAngle >= phiAngl3)
//         {
//             continue;
//         }
//         else if (nowPhiAngle >= -phiAngl2)
//         {
//             float distanceMax = (lengthToLoad + circleRadius_) / std::cos(nowPhiAngle);
//             if (laserDistance > distanceMin && laserDistance < distanceMax)
//             {
//                 return true;
//             }
//         }
//         else if (nowPhiAngle >= -phiAngl1)
//         {
//             float distanceMax = sensorCenterHeight / std::sin(nowPhiAngle);
//             if (laserDistance > distanceMin && laserDistance < distanceMax)
//             {
//                 return true;
//             }
//         } else
//         {
//             return false;
//         }
//     }

//     return false;
// }

bool CollisionWarning::moveRotatingBase(double targetTempAngle)
{
    bool wasLeechTargetAngle = false;

    double targetAngle = (isClockWise_ ? -targetTempAngle : targetTempAngle);
    double jointPos;
    bool isClockWiseLocal = (nowAngleSim_ > targetAngle);
    if ((isClockWiseLocal && nowAngleSim_ - moveAnglInCycle_ <= targetAngle)
    || (!isClockWiseLocal && nowAngleSim_ + moveAnglInCycle_ >= targetAngle))
    {       //if scan plane leeches target angle
        wasLeechTargetAngle = true;
        jointPos = targetAngle;
        if (isClockWiseLocal == isClockWise_)
        {
            isClockWise_ = !isClockWise_;
        }
    }
    else
    {
        jointPos = nowAngleSim_ + (isClockWiseLocal ? -moveAnglInCycle_ : moveAnglInCycle_);
    }
    nowAngleSim_ = jointPos;

    // Publish Position
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.velocity.resize(1);
    joint_state.name[0] = "rotating_base [rad, rad/sec]";
    joint_state.position[0] = jointPos + std::asin(1.0);
    joint_state.velocity[0] = pcl::deg2rad(rotateSpeed_);
    
    jointPubSim_.publish(joint_state);

    return wasLeechTargetAngle;
}

void CollisionWarning::moveModel(double xValue, double yValue, double zValue)
{
    geometry_msgs::Pose targetPose;
    targetPose.position.x = xValue;
    targetPose.position.y = yValue;
    targetPose.position.z = zValue;
    ROS_INFO_STREAM("xValue = " << xValue << ", yValue = " << yValue << ", zValue = " << zValue);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    targetPose.orientation.x = q.x();
    targetPose.orientation.y = q.y();
    targetPose.orientation.z = q.z();
    targetPose.orientation.w = q.w();

    gazebo_msgs::SetModelState setModelState;
    gazebo_msgs::ModelState    modelState;

    // Set ModelState INFO
    modelState.model_name      = warningVisModelName_;
    modelState.pose            = targetPose;
    modelState.reference_frame = referenceFrame_;

    // Servcie Call
    setModelState.request.model_state = modelState;
    size_t index = 0;
    while (!respawnClient_.call(setModelState))
    {
        index++;
        if (index >= NUM_OF_RETRY)
        {
            ROS_ERROR("failed to connect");
            return;
        }
    }
}

// double CollisionWarning::deg2rad(double degrees)
// {
//     return degrees * 4.0 * std::atan(1.0) / 180.0;
// }

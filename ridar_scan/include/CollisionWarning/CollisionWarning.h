#pragma once

// C++ specific includes
#include <cmath>
#include <algorithm>
#include <thread>
// #include <mutex>

// ROS specific includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>

#include "Common/ScanPlans2PointCloud.h"

#include "orion_rotating_base/sendCommand.h"
#include "orion_rotating_base/returnOrigin.h"

class CollisionWarning
{
protected:
    using SyncPolicySim = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::JointState, sensor_msgs::JointState>;
    using SyncPolicyNotSim = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::JointState>;

private:
    ros::NodeHandle* nodeHandlePtr_;
    bool isSimulation_;
    bool isOffLine_;
    ros::Publisher jointPubSim_;
    double nowAngleSim_;
    // ros::Subscriber jointSub_;
    message_filters::Synchronizer<SyncPolicySim>* syncSim_;
    message_filters::Synchronizer<SyncPolicyNotSim>* syncNotSim_;
    message_filters::Subscriber<sensor_msgs::LaserScan>  laserScanSub_;
    message_filters::Subscriber<sensor_msgs::JointState> jointSub_;
	message_filters::Subscriber<sensor_msgs::JointState> rotatingBaseStateSub_;
    ros::Publisher liveScanPub_;
    ros::Publisher collisionWarningResultPub_;
    ros::ServiceClient respawnClient_;
    ros::ServiceClient rotatingBaseActionClient_;

    double boomLength_;
    double sensorPosOnBoom_;
    double circleRadius_;
    double rotateSpeed_;
    double moveAnglInCycle_;
    bool isClockWise_;
    std::string referenceFrame_;
    std::string sensorModelFrameId_;
    std::string rope1LinkModelFrameId_;
    std::string warningVisModelName_;

    bool isFirst_;
    bool isFirstScan_;
    bool willPablishingCloud_;
    ros::Time previousTime_;
    ros::Time nowTime_;
    bool enablePCA_;

    // double loadXAxisSide_;
    // double loadYAxisSide_;
    double loadVertAxisSide_; // [m] vertically to ground plane
    double loadBoomParaAxisSide_; // [m] (parallel with boom)
    double loadBoomPerpAxisSide_; // [m] (perpendicular to boom)
    double warningMarginVert_; // [m]
    double warningMarginBoomPara_; // [m] (parallel with boom)
    double warningMarginBoomPerp_; // [m] (perpendicular to boom)
    double lengthCorrection_; // [m]
    double ropeLength_; // [m]
    double defaultYawJoint_;
    double defaultPitchJoint_;
    double boomLinkHeight_;
    // double upperLinkDefaultJoint_;
    // double boomLinkDefaultJoint_;

    std::vector<sensor_msgs::LaserScan> laserScanData_;
	std::vector<sensor_msgs::JointState> rotatingBaseStateData_;
    std::vector<SensorPoseStr> sensorPoseStrs_;
    // std::vector<double> translateZPositionData_;
	// std::vector<double> rotateYawAngleData_;

    int warningThreshold_;

    double loadAxisViewerFactor_;
    int loadAxisViewerNumOfPoints_;

    double correctionCWX_;
    double correctionCWY_;
    double correctionCCWX_;
    double correctionCCWY_;

    double sensorCylinderDistanceNotSim_;
    double sensorHeightNotSim_;
    // std::mutex mutex_;

    // double warningVizModelX_;
    // double warningVizModelWarnedY_;
    // double warningVizModelNotWarnedY_;
    // double warningVizModelZ_;

    // double warningVizModelWarnedZ_;
    // double warningVizModelNotWarnedZ_;

    void callbackSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg, const sensor_msgs::JointState::ConstPtr& jointMsg, const sensor_msgs::JointState::ConstPtr& angleMsg);
    void callbackNotSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg, const sensor_msgs::JointState::ConstPtr& angleMsg);
    bool warnOfCollision(pcl::PointCloud<pcl::PointXYZI>::Ptr& generatedCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& coloredCloud,
        double pitchJoint, double yawJoint);
    void resetVectors();
    // void generatePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& keyScan);
    // void normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const double refMin, const double refMax);
    pcl::PointXYZI range2Point(const float range, const float intensity, const float urgAngle, const double panAngle);
    // bool warnOfCollision(const sensor_msgs::LaserScan::ConstPtr& scanMsg, double thetaAngle, double lengthToLoad, double boomPitch);
    bool moveRotatingBase(double targetTempAngle);
    void moveModel(double xValue, double yValue, double zValue);
    void receiveScanDataThread();
    // double deg2rad(double degrees);

    // const double SUBSCRIBE_CYCLE = 0.01; // [s]
    const double SCAN_PLANE_DISTANCE = 0.15;
    const double SCAN_PLANE_DISTANCE_SIM = 0.0; // [m]
    const double CANT_SCAN_DISTANCE = 5.0; // [m]
    const double SCAN_DISTANCE_MAX = 250.0; // [m]
    const std::string PACKAGE_PATH = ros::package::getPath("ridar_scan");
    const double GROUND_PLANE_Z_VALUE = -1.9;
    const size_t NUM_OF_RETRY = 10;

public:
    CollisionWarning(ros::NodeHandle* nodeHandlePtr, bool isSimulation);
    ~CollisionWarning();
    void executeWarningSystem();
};

#pragma once

// C++ specific includes
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

// ROS specific includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>

// OpenCV specific includes
#include <opencv2/opencv.hpp>

// Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>

// #include "flir_ptu_driver/sendCommand.h"
// #include "flir_ptu_driver/initCommand.h"
#include "orion_rotating_base/sendCommand.h"
#include "orion_rotating_base/returnOrigin.h"

#include "ridar_scan/ExecuteLiveScan.h"

#include "pointcloud_integrator/scanMsg.h"
#include "pointcloud_integrator/keyScanMsg.h"

#include "Common/Pts.h"
#include "Common/ScanPlans2PointCloud.h"

#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"

// #include "model_msg_to_rotbase_joint_msg/PoseWithHeader.h"


class LiveScan
{

    public:
        // Constructor and Deconstructor 
        LiveScan(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr);
        ~LiveScan();

	bool executeScan(int speed);


    protected:
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::JointState>;
        // using SyncPolicySim = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::JointState, model_msg_to_rotbase_joint_msg::PoseWithHeader>;


    private:
        void readCalibFile();
        void dataReset();
        void receiveScanDataThread();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg, const sensor_msgs::JointState::ConstPtr& rotatingBaseMsg);
        // void callbackSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
        // void scanCallbackSim(const sensor_msgs::LaserScan::ConstPtr& scanMsg, const sensor_msgs::JointState::ConstPtr& rotatingBaseMsg, const model_msg_to_rotbase_joint_msg::PoseWithHeader::ConstPtr& pWithHeader);
        bool initPositionRotatingBase();
        bool moveRotatingBase(int speed, double endPos);
        bool executeLiveScanSrv(ridar_scan::ExecuteLiveScanRequest& req, ridar_scan::ExecuteLiveScanResponse& res);
        void pcl2Pts(const pcl::PointCloud<pcl::PointXYZI>& cloud, Pts& pts);
        void savePcd(const pcl::PointCloud<pcl::PointXYZI>& cloud);
	void saveImage(Pts& pts);
	void normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>& cloud, const double refMin, const double refMax);
        void pcl2KeyScanMsg(const pcl::PointCloud<pcl::PointXYZI>& cloud, pointcloud_integrator::keyScanMsg& ptsMsg, const int& id, bool normalizeRef);
        // pcl::PointXYZI range2Point(const float range, const float intensity, const float urgAngle, const double panAngle);
        bool moveSensorModel(const std::string targetModel, const std::string referenceFrame, const geometry_msgs::Pose& pose);
        geometry_msgs::Quaternion eulerAngles2Quaternion(double roll, double pitch, double yaw) const;
        void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;
        void showProgress();

        // Member variable
        ros::NodeHandle*                                     nodeHandlePtr_;
        ros::NodeHandle*                                     localNodeHandlePtr_;
        ros::Publisher                                       liveScanPub_;              // 結果である3Dスキャンの点群データを送信する配信者
        ros::Publisher                                       keyScanPub_;               // ある地点における3次元スキャンデータをpointcloud_integratorへ送信する配信者
        ros::Publisher                                       jointPubSim_;
        ros::ServiceClient                                   rotatingBaseActionClient_; // 回転台を動作させるクライアント 
        ros::ServiceServer                                   executeLiveScanSrv_;       // LiveScanを実行するためのサーバ
        ros::ServiceClient                                   respawnClient_;            // Gazeboにモデルの位置情報を与えるためのクライアント
        
        message_filters::Subscriber<sensor_msgs::LaserScan>  laserScanSub_;             // SICK LMS151のスキャンデータを受け取る購読者
        // ros::Subscriber                                      laserScanSubSim_;
	message_filters::Subscriber<sensor_msgs::JointState> rotatingBaseStateSub_;     // 回転台の情報として現在の位置と速度を受け取る購読者
	// message_filters::Subscriber<model_msg_to_rotbase_joint_msg::PoseWithHeader> poseSub_; // Gazebo上のモデルの位置情報を受け取る購読者
        message_filters::Synchronizer<SyncPolicy>*           sync_;                     // ApproximateTime Time Policyを用いてSICKとrotatingBaseのトピックをマッチングさせる
        // message_filters::Synchronizer<SyncPolicySim>*        syncSim_;                  // syncNotSim_のシミュレーション版

        bool                                                 isRotatingBaseWork_;       // 回転台が動作中であるかのフラグ 
        // double                                               scanStartRad_;             // スキャン開始位置を保持
        int                                                  actionNum_;                // スキャンの実行回数(executeScan関数の呼び出し回数)
        std::mutex                                           mutex_;                    // スレッド間で排他的なロックの仕組みを使用
        std::vector<sensor_msgs::LaserScan>                  laserScanData_;            // SICKから取得した全てのデータを保持
	std::vector<sensor_msgs::JointState>                 rotatingBaseStateData_;    // rotatingBaseから取得した全てのデータを保持
        // std::vector<geometry_msgs::Pose>                     pointAndQuat_;             // シミュレーションにおいて、センサの位置と角度を保持


        // Parameters
        int                                                  rotationSpeed_;            // LiveScan実行中のパンの移動速度[deg/sec]
        bool                                                 normalizeRef_;             // 得られた3次元点群データのintensityを正規化するかどうかのフラグ
        double                                               refMin_;                   // intensityを正規化する際の最小値
        double                                               refMax_;                   // intensityを正規化する際の最大値
        bool                                                 exportPcd_;                // LiveScanの結果をpcdファイルとして保存するかどうかのフラグ
        bool                                                 exportPts_;                // LiveScanの結果をptsファイルとして保存するかどうかのフラグ
        bool                                                 exportPly_;                // LiveScanの結果をplyファイルとして保存するかどうかのフラグ
        double                                               meshThresh_;               // plyファイル作成する時におけるメッシュ作成のしきい値

        bool                                                 enableTiltCompensation_;   // チルト補正を使うか
        double                                               calibPan0_;                //
        double                                               calibPan1_;                //
        double                                               calibTilt1_;               //

        enum CWorCCW
        {
                CLOCK_WISE,
                COUNTER_CLOCKWISE
        }                                                    cworccw_;                  // 時計回りか反時計回りか
        bool                                                 isOffLine_;                // オフライン処理か
        std::string                                          fileName_;                 // point cloud file name
        bool                                                 isSimulation_;             // シミュレーションかどうか
        double                                               rotateCycle_;              // シミュレーションにおいて、移動の周期
        std::string                                          sensorModelName_;          // Gazebo上のセンサモデルの名前
        std::string                                          referenceFrame_;           // Gazebo上のreference frame
        std::string                                          sensorModelFrameId_;       // tfのsensorModelのframe_id

        bool                                                 isMeasuring_;              //is measuring state
        double                                               panStart_;
        double                                               panEnd_;
        double                                               progressMeasuring_;
        double                                               progressUnit_;
        double                                               nowYawSim_;                // シミュレーションにおいて、現在のyaw
        // geometry_msgs::Pose                                  nowPoseSim_;               // シミュレーションにおいて、現在のPose
        double scanRange_;


        // Defines
        const double      SCAN_RANGE        = 100.0;                 // -SCAN_RANGE <= スキャンを実行 <= SCAN_RANGE 90.1[deg]
        const double      SCAN_RANGE_SIM    = 90.0;                  // -SCAN_RANGE <= スキャンを実行 <= SCAN_RANGE 90.1[deg]
        const double      MOVE_MARGIN       = 5.0;                   // 回転台を動かす際のSCAN_RANGEからのマージン [deg]
        const double      CANT_SCAN_DISTANCE = 5.0;                  // SICKが計測不可能な距離 [m]
        const double      SCAN_DISTANCE_MAX = 250.0;                 // SICKが計測可能な最大距離 [m]
        const size_t      ONE_SCAN_LINE_NUM = 1081;
        const double      SCAN_PLANE_DISTANCE = 0.15;                // 回転台の回転軸とSICKのスキャン平面との距離
        const double      PI                = 3.141592653589793;
        const double      SEMICIRCLE_RAD    = pcl::deg2rad(180.0);
        const std::string PACKAGE_PATH      = ros::package::getPath("ridar_scan");
        const double SCAN_PLANE_DISTANCE_SIM = 0.0;
        const size_t NUM_OF_RETRY = 10;

};

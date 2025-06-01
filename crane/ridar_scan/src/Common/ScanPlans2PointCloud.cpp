#include "Common/ScanPlans2PointCloud.h"
#include <Eigen/Dense>

ScanPlans2PointCloud::ScanPlans2PointCloud()
{
}

ScanPlans2PointCloud::~ScanPlans2PointCloud()
{
}

// for live scan
void ScanPlans2PointCloud::generatePointCloud(pcl::PointCloud<pcl::PointXYZI>& keyScan,
        const std::vector<sensor_msgs::LaserScan>& laserScanData, const std::vector<sensor_msgs::JointState>& rotatingBaseStateData,
        const size_t scanStart, const size_t scanEnd,
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const bool enableTiltCompensation, const double calibPan0, const double calibPan1, const double calibTilt1,
        const double scanRange,
        const bool normalizeRef, const double refMin, const double refMax)
{
    SensorPoseStr zeroPoseStr(0.0,0.0,0.0,0.0);
    std::vector<SensorPoseStr> sensorPoseStrs{zeroPoseStr};
    generatePointCloud(keyScan,
        laserScanData, rotatingBaseStateData,
        scanStart, scanEnd,
        cantScanDistance, scanDistanceMax, isSimulation, scanPlaneDistance,
        enableTiltCompensation, calibPan0, calibPan1, calibTilt1,
        sensorPoseStrs,
        scanRange,
        normalizeRef, refMin, refMax);
}

// for collision warning
void ScanPlans2PointCloud::generatePointCloud(pcl::PointCloud<pcl::PointXYZI>& keyScan,
        const std::vector<sensor_msgs::LaserScan>& laserScanData, const std::vector<sensor_msgs::JointState>& rotatingBaseStateData,
        const size_t scanStart, const size_t scanEnd,
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const std::vector<SensorPoseStr>& sensorPoseStrs,
        const double scanRange,
        const bool normalizeRef, const double refMin, const double refMax)
{
    generatePointCloud(keyScan,
        laserScanData, rotatingBaseStateData,
        scanStart, scanEnd,
        cantScanDistance, scanDistanceMax, isSimulation, scanPlaneDistance,
        false, 0.0, 0.0, 0.0,
        sensorPoseStrs,
        scanRange,
        normalizeRef, refMin, refMax);
}

// private
void ScanPlans2PointCloud::generatePointCloud(pcl::PointCloud<pcl::PointXYZI>& keyScan,
        const std::vector<sensor_msgs::LaserScan>& laserScanData, const std::vector<sensor_msgs::JointState>& rotatingBaseStateData,
        const size_t scanStart, const size_t scanEnd,
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const bool enableTiltCompensation, const double calibPan0, const double calibPan1, const double calibTilt1,
        const std::vector<SensorPoseStr>& sensorPoseStrs,
        const double scanRange,
        const bool normalizeRef, const double refMin, const double refMax)
{
    // スキャンデータから変換されたある地点における3次元データを保持
    // pcl::PointCloud<pcl::PointXYZI> keyScan;

    // SICKから得られたスキャンデータを割り振る処理
    for (size_t i = scanStart; i <= scanEnd; ++i) {
        sensor_msgs::LaserScan laserScan = laserScanData[i];
        sensor_msgs::JointState rotatingBaseState = rotatingBaseStateData[i];

        // スキャンデータが取得された際のパン角を更新
        double panAngle = rotatingBaseState.position[0];

        // scanRange_以外の範囲で取得したスキャンデータは変換に考慮しない
        // if (panAngle > (isSimulation_ ? pcl::deg2rad(scanRange_) : scanRange_) || panAngle < (isSimulation_ ? pcl::deg2rad(-1.0 * scanRange_) : -1.0 * scanRange_))
        if (scanRange != 0.0 && (panAngle > scanRange || panAngle < -scanRange))
            continue;

        // SICKのスキャン開始位置[rad]および角度分解能[rad]の実測値を保持
        float urgStart, urgIncrement;
        urgStart = laserScan.angle_min;
        urgIncrement = laserScan.angle_increment;

        for (size_t j = 0; j < laserScan.ranges.size(); ++j) {
            // SICKのスキャン角度を更新
            float urgAngle = urgStart + urgIncrement * j;

            // 距離データと反射強度の値を取得
            float range = laserScan.ranges[j];
            float intensity = laserScan.intensities[j];

            // 距離データから3次元位置を計算
            pcl::PointXYZI point = range2Point(range, intensity,
                                    urgAngle, panAngle,
                                    cantScanDistance, scanDistanceMax, isSimulation, scanPlaneDistance,
                                    enableTiltCompensation, calibPan0, calibPan1, calibTilt1,
                                    sensorPoseStrs.size() == 1 ? sensorPoseStrs[0] : sensorPoseStrs[i]);

            keyScan.push_back(point);
        }
    }

    if (normalizeRef) {
        // リフレクタンス値の正規化
        normalizeReflectance(keyScan, refMin, refMax);
    }
}

// private
pcl::PointXYZI ScanPlans2PointCloud::range2Point(const float range, const float intensity,
        const float urgAngle, const double panAngRad, 
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const bool enableTiltCompensation, const double calibPan0, const double calibPan1, const double calibTilt1,
        const SensorPoseStr& sensorPoseStr)
{
    pcl::PointXYZI point;
    // double panAngRad = (isSimulation_ ? panAngle : pcl::deg2rad(panAngle));

    // 距離データの値が範囲外である場合
    if (range <= cantScanDistance || range > scanDistanceMax) {
        // 計測することができなかったデータとして全ての値をNANとする
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = NAN;

        return point;
    }

    double xForUv = (isSimulation ? scanPlaneDistance : range * cos(urgAngle));
    double yForUv = (isSimulation ? range * sin(urgAngle) : scanPlaneDistance);
    double zForUv = (isSimulation ? -range * cos(urgAngle) : -range * sin(urgAngle));
    Eigen::Vector3d uv0(xForUv, yForUv, zForUv);
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(panAngRad + sensorPoseStr.yaw, Eigen::Vector3d::UnitZ());

    if(enableTiltCompensation) {
        double p = (panAngRad - calibPan0) / (calibPan1 - calibPan0);
        double tilt = (1 - p) * calibTilt1;
        rotation = Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()) * rotation;
    }

    Eigen::Vector3d xyz = rotation * uv0;

    point.x = xyz.x() + sensorPoseStr.posx;
    point.y = xyz.y() - sensorPoseStr.posy;
    point.z = xyz.z() + sensorPoseStr.posz;
    point.intensity = intensity;

    return point;
}

// private
void ScanPlans2PointCloud::normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>& cloud, const double refMin, const double refMax)
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

// C++ specific includes
#include <vector>

// ROS specific includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>

struct SensorPoseStr
{
    const double posx;
    const double posy;
    const double posz;
    const double yaw;
    SensorPoseStr(const double xValue, const double yValue, const double zValue, const double yawValue)
      : posx(xValue),
        posy(yValue),
        posz(zValue),
        yaw(yawValue)
    {
    }
    ~SensorPoseStr()
    {
    }
};

class ScanPlans2PointCloud
{
private:
    static constexpr bool NORMALIZE_REF = false;
    static constexpr double REF_MIN = 300.0;
    static constexpr double REF_MAX = 900.0;

    static void generatePointCloud(pcl::PointCloud<pcl::PointXYZI>& keyScan,
        const std::vector<sensor_msgs::LaserScan>& laserScanData, const std::vector<sensor_msgs::JointState>& rotatingBaseStateData,
        const size_t scanStart, const size_t scanEnd,
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const bool enableTiltCompensation, const double calibPan0, const double calibPan1, const double calibTilt1,
        const std::vector<SensorPoseStr>& sensorPoseStrs,
        const double scanRange = 0.0,
        const bool normalizeRef = NORMALIZE_REF, const double refMin = REF_MIN, const double refMax = REF_MAX);
    static pcl::PointXYZI range2Point(const float range, const float intensity,
        const float urgAngle, const double panAngRad, 
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const bool enableTiltCompensation, const double calibPan0, const double calibPan1, const double calibTilt1,
        const SensorPoseStr& sensorPoseStr);
    static void normalizeReflectance(pcl::PointCloud<pcl::PointXYZI>& cloud, const double refMin, const double refMax);

public:
    ScanPlans2PointCloud();
    ~ScanPlans2PointCloud();

    // for live scan
    static void generatePointCloud(pcl::PointCloud<pcl::PointXYZI>& keyScan,
        const std::vector<sensor_msgs::LaserScan>& laserScanData, const std::vector<sensor_msgs::JointState>& rotatingBaseStateData,
        const size_t scanStart, const size_t scanEnd,
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const bool enableTiltCompensation, const double calibPan0, const double calibPan1, const double calibTilt1,
        const double scanRange = 0.0,
        const bool normalizeRef = NORMALIZE_REF, const double refMin = REF_MIN, const double refMax = REF_MAX);

    // for collision warning
    static void generatePointCloud(pcl::PointCloud<pcl::PointXYZI>& keyScan,
        const std::vector<sensor_msgs::LaserScan>& laserScanData, const std::vector<sensor_msgs::JointState>& rotatingBaseStateData,
        const size_t scanStart, const size_t scanEnd,
        const double cantScanDistance, const double scanDistanceMax, const bool isSimulation, const double scanPlaneDistance,
        const std::vector<SensorPoseStr>& sensorPoseStrs,
        const double scanRange = 0.0,
        const bool normalizeRef = NORMALIZE_REF, const double refMin = REF_MIN, const double refMax = REF_MAX);
};

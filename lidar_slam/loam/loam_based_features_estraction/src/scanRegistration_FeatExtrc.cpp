#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include <pcl/ros/conversions.h>
// pcl fromROSMsg() has changed, need to include <pcl_conversions/pcl_conversions.h> header
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "matplotlibcpp.h"

#include <cmath>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <limits>

// #include </home/aisl2/catkin_ws/src/loam/loam_back_and_forth/src/matplotlibcpp.h>

namespace plt = matplotlibcpp;

std::vector<int> laserAngleCur_array; 
std::vector<int> laserRotDir_array; 

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double initTime;
double timeStart;
double last_sweep_time;
double diff_sweep_time;
double timeLasted;
bool systemInited = false;

double timeScanCur = 0;
double timeScanLast = 0;

int laserRotDir = 1;
float laserAngleLast = 0;
float laserAngleCur = 0;

int skipFrameNum = 3;
int skipFrameCount = 0;

int counter_sweep = 0;
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

pcl::PointCloud<pcl::PointXYZHSV>::Ptr ExcludedPoints(new pcl::PointCloud<pcl::PointXYZHSV>());

sensor_msgs::PointCloud2 laserCloudExtreCur2;
sensor_msgs::PointCloud2 laserCloudLast2;

ros::Publisher* pubLaserCloudExtreCurPointer;
ros::Publisher* pubLaserCloudLastPointer;
ros::Publisher* pubLaserExcludedPointsPointer;
ros::Publisher* pubLaserCloudCornerPointsSharpPointer;
ros::Publisher* pubLaserCloudSurfPointsFlatPointer;
ros::Publisher* pubLaserCloudCornerPointsLessSharpPointer;
ros::Publisher* pubLaserCloudSurfPointsLessFlatPointer;

ros::Publisher* pubLaserCloudGroundPointsPointer;
ros::Publisher* pubLaserCloudWall1PointsPointer;
ros::Publisher* pubLaserCloudWall2PointsPointer;
ros::Publisher* pubLaserCloudWall3PointsPointer;

double min_laser_range = .3;
double max_laser_range = 500;

int cloudSortInd[1200];
int cloudNeighborPicked[1200];

std::vector<float> laserCloud_x1;  
std::vector<float> laserCloud_delta_x;  
std::vector<float> laserCloud_x2;  
std::vector<float> laserCloud_y1;  
std::vector<float> laserCloud_y2;  
std::vector<float> laserCloud_delta_y;  
std::vector<float> laserCloud_z1;  
std::vector<float> laserCloud_z2;  
std::vector<float> laserCloud_delta_z;  
std::vector<float> laserCloud_tan;  


int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 400;
bool imuInited = false;

float imuRollStart, imuPitchStart, imuYawStart;
float imuRollCur, imuPitchCur, imuYawCur;

float imuVeloXStart, imuVeloYStart, imuVeloZStart;
float imuShiftXStart, imuShiftYStart, imuShiftZStart;
float imuVeloXCur, imuVeloYCur, imuVeloZCur;
float imuShiftXCur, imuShiftYCur, imuShiftZCur;

float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

//IMU information, including time, Euler angle, acceleration, velocity, displacement:

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};


// for newSweep
bool newSweep = false;
#define ZERO_THRESHOLD 6
#define TIME_THRESHOLD 30
// Time threshold to ignore sudden changes (in seconds)
// TIME_THRESHOLD = 50 # time for 4
// TIME_THRESHOLD = 30 # time for 5


double last_yaw;
ros::Publisher pub;
bool assemble_scans = false;
ros::Time start_time;
int map_count = 0;
double prev_yaw;
int direction;
ros::Time last_change_time;
int cycle_count = 0;
std::vector<double> yaw_angles, pitch_angles, roll_angles, timestamps;
bool crossed_zero_pos = false;
bool crossed_zero_neg = false;


//            Start
//              |
//       +-------------+
//       | imuHandler()|
//       +-------------+
//              |
//              v
//  +----------------------+
//  | AccumulateIMUShift() |
//  +----------------------+
//              |
//              v                             
//  +----------------------+          +----------------+
//  | Distortion Removal   |<---------| IMU Measurements|
//  +----------------------+          +----------------+
//              |                               
//              v                               
//  +-----------------+                         
//  | ShiftToStartIMU |                         
//  +-----------------+                         
//              |                               
//              v                               
//  +-----------------+                         
//  | VeloToStartIMU  |                         
//  +-----------------+                        
//              |                               
//              v                               
//  +----------------------------+              
//  | TransformToStartIMU(p)     |              
//  +----------------------------+              
//              |                               
//              v                               
//           End                             
//======================================================================
// 5. Distortion Removal
// After receiving the IMU information, the point cloud distortion is processed through the IMU information,
// and laserCloudHandler (detailed analysis in the next) is called.

// 1. Displacement Distortion Calculation (ShiftToStartIMU)
// Calculate the displacement distortion of the points in the point cloud in the local coordinate system
// relative to the first starting point due to acceleration and deceleration.
// First, calculate the displacement in the world coordinate system:
// δS_w = S_cur - S_start - V_start * T

// Then transform to the local coordinate system:
// δS_scan = R_z^-1 * R_x^-1 * R_y^-1 * δS_w


// Function to publish empty point clouds
void publishEmptyCloud(ros::Publisher& pub, double timeScanCur) {
    pcl::PointCloud<pcl::PointXYZHSV> emptyCloud;
    sensor_msgs::PointCloud2 laserCloudPoints;
    pcl::toROSMsg(emptyCloud, laserCloudPoints);
    laserCloudPoints.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudPoints.header.frame_id = "world";
    pub.publish(laserCloudPoints);
}


void ShiftToStartIMU()
{
  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

// 2. Velocity Distortion Calculation (VeloToStartIMU)
// Calculate the velocity distortion (increment) of the points in the point cloud in the local coordinate system
// relative to the first starting point due to acceleration and deceleration.
// First, calculate the velocity in the world coordinate system:
// δV_w = V_cur - V_start
// Then transform to the local coordinate system:
// δV_scan = R_z^-1 * R_x^-1 * R_y^-1 * δV_w

void VeloToStartIMU()
{
  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}


// 3. Remove Distortion (TransformToStartIMU)
// Transfer the point cloud coordinates to the world coordinate system through the rotation relationship of IMU
// (X_{imu} -> X_w), and then transfer from the world coordinate system to the initial coordinate system of the
// current frame through the angle at the beginning of the current frame (X_w -> X_{scan}).
// Finally, add the displacement error calculated in ShiftToStartIMU to complete the compensation.

void TransformToStartIMU(pcl::PointXYZHSV *p)
{
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}
// ========================================
// IMU Callback Function
// This function processes incoming IMU data to extract orientation (roll, pitch, yaw) and linear acceleration.
// It then adjusts the acceleration to account for gravity and stores the processed data for further use.

// IMU Shift Accumulation Function (AccumulateIMUShift)
// Integrate Velocity and Displacement
// This function uses the stored IMU data to calculate the displacement and velocity of the sensor in the world coordinate system.
// It transforms the acceleration from the IMU frame to the world frame, then integrates to compute velocity and displacement.


// 2. Integrate Velocity and Displacement (AccumulateIMUShift) Background and basic
// In three-dimensional space, the rotation matrix is decomposed into the xyz axes.
// When defining rotation in terms of zxy, we have:
// X_w^T = X_imu^T * R_z^-1 * R_x^-1 * R_y^-1

// After obtaining the acceleration in the world coordinate system,
// the displacement and velocity in the world coordinate system can be calculated as:
// S_last = S_back + V_back * T + 1/2* a * T^2
// V_last = V_back * T + a * T

void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  // Transform acceleration to the world coordinate system
  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < 0.1) {
    // Integrate to calculate displacement
    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;
    // Integrate to calculate velocity
    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}





// 4. IMU Callback Background
// 1. Receive Angle and Acceleration Information from IMU
// For the IMU, it is essential to eliminate the influence of gravity acceleration on the measurement.
// The gravity vector in the navigation frame is given by:
// g^n = [0, 0, -9.81]

// The rotation matrix expressed in Euler angles is:
// R_b^n = R_z * R_y * R_x
// Therefore,
// R_n^b = (R_b^n)^T

// The error caused by gravity acceleration can be calculated as:
// [sin(θ) * 9.81, -sin(ψ) * sin(θ) * 9.81, -cos(ψ) * cos(θ) * 9.81]

// Note that the IMU coordinate system is a right-handed coordinate system with:
// x-axis forward, y-axis to the left, and z-axis upward.

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // Adjust acceleration to account for gravity
  float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec() - 0.1068;
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  AccumulateIMUShift();
}




void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract yaw from odometry message
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    yaw = yaw * 180.0 / M_PI; // Convert radians to degrees
    pitch = pitch * 180.0 / M_PI; // Convert radians to degrees
    roll = roll * 180.0 / M_PI; // Convert radians to degrees

    // Append the yaw angle and current time to the lists
    yaw_angles.push_back(yaw);
    pitch_angles.push_back(pitch);
    // roll_angles.push_back(roll);
    // timestamps.push_back(ros::Time::now().toSec());

    // Initialize start_time on first callback
    if (start_time.isZero()) {
        start_time = ros::Time::now();
    }

    // Check if enough time has passed since the last direction change
    if (!last_change_time.isZero()) {
        double time_since_last_change = (ros::Time::now() - last_change_time).toSec();
        if (time_since_last_change < TIME_THRESHOLD && (std::abs(yaw) < 10 || std::abs(yaw) > 150)) {
            ROS_INFO("Ignoring sudden direction change at yaw: %.1f", yaw);
            prev_yaw = yaw;
            return;
        }
    }

    // Check if enough time has passed since the last direction change
    if (prev_yaw != 0) {
        double yaw_diff = yaw - prev_yaw;

        // Detect zero crossing with hysteresis and threshold
        if (prev_yaw < 0 && yaw >= -ZERO_THRESHOLD) {
            crossed_zero_pos = true;
            last_change_time = ros::Time::now();
            ROS_INFO("crossed_zero_pos, prev_yaw: %.1f, yaw: %.1f, abs(yaw_diff): %.1f", prev_yaw, yaw, std::abs(yaw_diff));
            ROS_INFO("============================================================== Completed a 180-degree cycle: %d", cycle_count);
        }

        if (prev_yaw > 0 && yaw <= ZERO_THRESHOLD) {
            crossed_zero_neg = true;
            last_change_time = ros::Time::now();
            ROS_INFO("crossed_zero_neg, prev_yaw: %.1f, yaw: %.1f, abs(yaw_diff): %.1f", prev_yaw, yaw, std::abs(yaw_diff));
            ROS_INFO("============================================================== Completed a 180-degree cycle: %d", cycle_count);
        }
    }

    ROS_INFO("yaw: %.1f, zero_pos: %d, zero_neg: %d, count: %d", yaw, crossed_zero_pos, crossed_zero_neg, cycle_count);

    // Check for full 360-degree cycle
    if (crossed_zero_pos && crossed_zero_neg) {
        cycle_count++;
        newSweep = true;
        ROS_INFO("============================================================== Completed a 360-degree cycle: %d", cycle_count);
        crossed_zero_pos = false;
        crossed_zero_neg = false;
        ros::Time end_time = ros::Time::now();
        ROS_INFO("Start T: %.2f, End T: %.2f", start_time.toSec(), end_time.toSec());
        ROS_INFO("map_count %d ", map_count);
        start_time = end_time;

      }

    prev_yaw = yaw;
}



void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{

  //Discard the first 20 point cloud data:

  if (!systemInited) {
    initTime = laserCloudIn2->header.stamp.toSec();
    imuPointerFront = (imuPointerLast + 1) % imuQueLength;
    systemInited = true;
  }

// Get the current point cloud time (the timestamp in ROS is generally: MsgPtr->header.stamp.toSec()
  timeScanLast = timeScanCur;
  timeScanCur = laserCloudIn2->header.stamp.toSec();
  timeLasted = timeScanCur - initTime;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
  int cloudSize = laserCloudIn->points.size();

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>(cloudSize, 1));
  for (int i = 0; i < cloudSize; i++) {
    laserCloud->points[i].x = laserCloudIn->points[i].x;
    laserCloud->points[i].y = laserCloudIn->points[i].y;
    laserCloud->points[i].z = laserCloudIn->points[i].z;
    laserCloud->points[i].h = timeLasted;
    laserCloud->points[i].v = 0;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
  }


//--------------------------------------------------------------------
// Calculate the 2d lidar rotating angle and detect the complete sweep
//---------------------------------------------------------------------
  // bool newSweep = false;
    // std::cout << " newSweep: " << newSweep << std::endl;
  laserAngleLast = laserAngleCur;
// LOAM
  // laserAngleCur = atan2(laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y, 
  //                       laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x);
  // std::cout << " laserAngleCur: " << laserAngleCur* 180.0 / M_PI << " delta Y: " << round((laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y)*100)/100 << " delta X: " << round((laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x)*100)/100 << std::endl;
  // std::cout << " y2 " << round((laserCloud->points[cloudSize - 1].y)*100)/100 << " y1: " << round((laserCloud->points[0].y)*100)/100 << " X1: " << round((laserCloud->points[cloudSize - 1].x)*100)/100 << " X2: " << round((laserCloud->points[0].x)*100)/100 << std::endl;

  // if (laserAngleLast > 0 && laserRotDir == 1 && laserAngleCur < laserAngleLast){
  //   laserRotDir = -1;
  //   newSweep = true;
  //   //  std::cout << " Last: " << round(laserAngleLast*100000)/100000 << " >  0  &&  > Cur: " << round(laserAngleCur*100000)/100000 << " laserRotDir == 1" <<std::endl;

  // } else if (laserAngleLast < 0 && laserRotDir == -1 && laserAngleCur > laserAngleLast ) {
  //   laserRotDir = 1;
  //   newSweep = true;
  //   //  std::cout << " Last: " << round(laserAngleLast*100000)/100000 << " <  0  &&  < Cur: " << round(laserAngleCur*100000)/100000 << " laserRotDir == -1" <<std::endl;
  // }

// ROS_INFO ("%d %d", laserCloudLast2.width, laserCloudExtreCur2.width);
//  std::cout << " Last: " << laserAngleLast << " Cur: " << laserAngleCur << " laserRotDir: " << laserRotDir <<std::endl;

// save the data in vector to plot using matpolotlib
  laserCloud_x1.push_back(laserCloud->points[0].x);
  laserCloud_y1.push_back(laserCloud->points[0].y);
  laserCloud_z1.push_back(laserCloud->points[0].z);
  laserCloud_x2.push_back(laserCloud->points[cloudSize - 1].x);
  laserCloud_y2.push_back(laserCloud->points[cloudSize - 1].y);
  laserCloud_z2.push_back(laserCloud->points[cloudSize - 1].z);
  laserCloud_tan.push_back(laserAngleCur);
  laserCloud_delta_y.push_back(laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y);
  laserCloud_delta_x.push_back(laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x);
  laserCloud_delta_z.push_back(laserCloud->points[cloudSize - 1].z - laserCloud->points[0].z);
  std::vector<int> laserAngleLast_array; 
  laserAngleLast_array.push_back(laserAngleLast);
  laserAngleCur_array.push_back(laserAngleCur);
  laserRotDir_array.push_back(laserRotDir);


//--------------------------------------------------------------------
// If one sweep is completed store the imu data in imuTrans
//---------------------------------------------------------------------
  if (newSweep) {
    newSweep = false;
    timeStart = timeScanLast - initTime;
    diff_sweep_time =  timeStart - last_sweep_time;
  
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
   
    // Starting point Euler angle
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    //Euler angle of the last point
    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

    // distortion displacement of the last point relative to the first point
    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    // distortion velocity of the last point relative to the first point
    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    *laserCloudExtreCur += *laserCloudLessExtreCur;


map_count = map_count +1;
std::cout << "PCDWriter" << std::endl;
// save the pointcloud map as pcd
std::string file_directory;  // Declare the file_directory variable
file_directory = "/media/aisl2/aisl_data/catkin_ws/src/loam/loam_based_features_estraction/";
pcl::PCDWriter writer;
std::stringstream ss;
ss << file_directory << "orignal_data" << map_count << ".pcd";

writer.write<pcl::PointXYZHSV> (ss.str (), *laserCloudExtreCur, false);



   //  int cloudSize = laserCloudIn->points.size();
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2);

    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    // laserCloudLast2.header.frame_id = "camera";
    laserCloudLast2.header.frame_id = "world";

    
    
    laserCloudExtreCur->clear();
    laserCloudLessExtreCur->clear();
    imuTrans->clear();

    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;

    counter_sweep = counter_sweep +1;

   if (counter_sweep < -6){
      plt::figure();
      plt::suptitle("LaserAngle Cur, Last, and RotDir"); // add a title
      plt::subplot(1, 1, 1); plt::plot(laserAngleLast_array, {{"label", "laserAngleLast_array"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(1, 1, 1); plt::plot(laserAngleCur_array, {{"label", "laserAngleCur_array"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(1, 1, 1); plt::plot(laserRotDir_array, {{"label", "laserRotDir_array"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");

      // plt::subplot(2, 2, 2); plt::plot(cloudSortInd_vec); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");

      plt::figure();
      plt::suptitle("laserCloud_delta_x laserCloud_delta_y laserCloud_tan "); // add a title
      plt::subplot(1, 1, 1); plt::plot(laserCloud_delta_x, {{"label", "laserCloud_delta_x"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(1, 1, 1); plt::plot(laserCloud_delta_y, {{"label", "laserCloud_delta_y"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(1, 1, 1); plt::plot(laserCloud_tan, {{"label", "laserCloud_tan"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");


      plt::figure();
      plt::suptitle("x, y, z"); // add a title
      plt::subplot(3, 1, 1); plt::plot(laserCloud_x1, {{"label", "x1"}, {"marker", "o"}, {"linestyle", "--"}});  plt::plot(laserCloud_x2, {{"label", "x2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(3, 1, 2); plt::plot(laserCloud_y1, {{"label", "y1"}, {"marker", "o"}, {"linestyle", "--"}});  plt::plot(laserCloud_y2, {{"label", "y2"}, {"marker", "o"}, {"linestyle", "--"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(3, 1, 3); plt::plot(laserCloud_z1, {{"label", "z1"}, {"marker", "o"}, {"linestyle", "--"}});  plt::plot(laserCloud_z2, {{"label", "z2"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");

      plt::figure();
      plt::suptitle("laserCloud_delta_x laserCloud_delta_y laserCloud_delta_z "); // add a title
      plt::subplot(3, 1, 1); plt::plot(laserCloud_delta_x, {{"label", "laserCloud_delta_x"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(3, 1, 2); plt::plot(laserCloud_delta_y, {{"label", "laserCloud_delta_y"}, {"marker", "o"}, {"linestyle", "--"}});  plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
      plt::subplot(3, 1, 3); plt::plot(laserCloud_delta_z, {{"label", "laserCloud_delta_z"}, {"marker", "o"}, {"linestyle", "--"}});   plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");

      plt::show();
      counter_sweep = 0;
      }
      last_sweep_time = timeStart;
  }
  imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
  imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
  imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;


//--------------------------------------------------------------------
// 5. Distortion Removal
// After receiving the IMU information, the point cloud distortion is processed through the IMU information, and laserCloudHandler 
//---------------------------------------------------------------------
// If IMU data is received, use IMU to correct point cloud distortion
  if (imuPointerLast >= 0) {
  // Find where the imu time exceeds the lidar time:
    while (imuPointerFront != imuPointerLast) {
  /* Find the IMU position where the timestamp of the point cloud 
    is smaller than the timestamp of the IMU: imuPointerFront*/
      if (timeScanCur < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }


    /* If the time stamp of the imu leading lidar is not found, imuPointerFront==imtPointerLast 
    can only use the latest received IMU speed, displacement, and Euler angle as the speed, 
    displacement, and Euler angle of the current point
    */

    if (timeScanCur > imuTime[imuPointerFront]) {
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];

      imuVeloXCur = imuVeloX[imuPointerFront];
      imuVeloYCur = imuVeloY[imuPointerFront];
      imuVeloZCur = imuVeloZ[imuPointerFront];

      imuShiftXCur = imuShiftX[imuPointerFront];
      imuShiftYCur = imuShiftY[imuPointerFront];
      imuShiftZCur = imuShiftZ[imuPointerFront];
    } 
    //  you find the timestamp of the imu ahead of the lidar, perform linear interpolation on the current motion information
    else {
    //Find the IMU position whose point cloud time stamp is smaller than the IMU time stamp, 
    // then the point must be between imuPointerBack and imuPointerFront, based on this linear interpolation, 
    // calculate the speed, displacement and Euler angle of the point cloud point
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
    // Calculate the weight distribution ratio according to the time distance, that is, linear interpolation
      float ratioFront = (timeScanCur - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeScanCur) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
      } else {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }

      imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
      imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
      imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

      imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
      imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
      imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
    }
  }

// If it is the first point, remember the velocity, displacement and Euler angle of the starting position of the point cloud
  if (!imuInited) {
    imuRollStart = imuRollCur;
    imuPitchStart = imuPitchCur;
    imuYawStart = imuYawCur;

    imuVeloXStart = imuVeloXCur;
    imuVeloYStart = imuVeloYCur;
    imuVeloZStart = imuVeloZCur;

    imuShiftXStart = imuShiftXCur;
    imuShiftYStart = imuShiftYCur;
    imuShiftZStart = imuShiftZCur;

    imuInited = true;
  }


/*Calculate the displacement velocity distortion of each point relative to the first point 
due to acceleration and deceleration non-uniform motion, and re-compensate and 
correct the position information of each point in the point cloud
 (the following functions will be analyzed separately
*/
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * (timeLasted - timeStart);
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * (timeLasted - timeStart);
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * (timeLasted - timeStart);

  ShiftToStartIMU();

  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

   VeloToStartIMU();

  for (int i = 0; i < cloudSize; i++) {
    TransformToStartIMU(&laserCloud->points[i]);
  }


  // Divide all point clouds into four categories based on curvature of each point cloud point
  // pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  // pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  // cornerPointsSharp->push_back(laserCloud->points[0]);
  // cornerPointsLessSharp->push_back(laserCloud->points[cloudSize - 1]);

//--------------------------------------------------------------
// Calculate the curvature of each point cloud point
//--------------------------------------------------------------
    //Get the number of points in the valid range
  for (int i = 5; i < cloudSize - 5; i++) { //Use the five points before and after each point 
  // to calculate the curvature, so the first five and the last five points are skipped
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
    //curvature calculation
    laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
    // diff_array.push_back(laserCloud->points[i].s);

  }

//--------------------------------------------------------------
// exclude points that are easily blocked by the slope and outliers
//----------------------------------------------------------------

//  Filter feature points
/* Pick points, exclude points that are easily blocked by the slope and outliers, 
some points are easily blocked by the slope and outliers may appear by chance, 
these situations may lead to two scans not being seen at the same time*/
  for (int i = 5; i < cloudSize - 6; i++) { //The difference with the next point, so subtract 6
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
//Calculate the sum of squares of the distance between the effective curvature point and the next point
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

/*If the distance between the current point and the latter point is too large (greater than 0.1),
 it is necessary to judge the size of the angle formed by the two laser points and the radar, 
 the approach taken is to pull the distant laser point back to the near (that is, the three points form an isosceles triangle),
 under the premise of a small angle, the angle is approximated by the ratio of the side lengths, if the angle is small, 
it means that the plane where the laser point is located is approximately parallel to the laser line, discarding the distant laser point.*/
    if (diff > 0.05) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.005) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;

          ExcludedPoints->push_back(laserCloud->points[i - 5]);
          ExcludedPoints->push_back(laserCloud->points[i - 4]);
          ExcludedPoints->push_back(laserCloud->points[i - 3]);
          ExcludedPoints->push_back(laserCloud->points[i - 2]);
          ExcludedPoints->push_back(laserCloud->points[i - 1]);
          ExcludedPoints->push_back(laserCloud->points[i ]);

        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.005) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;

          // ExcludedPoints->push_back(laserCloud->points[i + 1]);
          // ExcludedPoints->push_back(laserCloud->points[i + 2]);
          // ExcludedPoints->push_back(laserCloud->points[i + 3]);
          // ExcludedPoints->push_back(laserCloud->points[i + 4]);
          // ExcludedPoints->push_back(laserCloud->points[i + 5]);
          // ExcludedPoints->push_back(laserCloud->points[i + 6]);

        }
      }
    }

/*If the distance between the current point and the two points before and after exceeds 0.0002 of the current depth,
it will be regarded as an outlier point and discarded.*/
    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
      cloudNeighborPicked[i] = 1;
      // ExcludedPoints->push_back(laserCloud->points[i]);

    }
  }

// =========================================================
//Extract feature points
// =========================================================

// Divide all point clouds into four categories:
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
   pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPoints(new pcl::PointCloud<pcl::PointXYZHSV>());


//Divide the curvature points of each scan into 4 equal parts
  int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 
                        6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 
                      5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

  for (int i = 0; i < 4; i++) {
    int sp = startPoints[i];
    int ep = endPoints[i];

    // std::cout << " -------------------------------------------- " << std::endl;
    // std::cout << " startPoints[i] " << startPoints[i] << std::endl;
    // std::cout << " endPoints[i]   " << endPoints[i] << std::endl;
   


    for (int j = sp + 1; j <= ep; j++) {
      for (int k = j; k >= sp + 1; k--) {
        if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {
          int temp = cloudSortInd[k - 1];
          cloudSortInd[k - 1] = cloudSortInd[k];
          cloudSortInd[k] = temp;
        }
      }
    }



/* Select 20 points with relatively large curvature, and two points with large curvature,
and discard the points close to the feature points after selection
*/
//--------------------------------------------------------------
// divide the points in cornerPointsSharp and  cornerPointsLessSharp
//----------------------------------------------------------------
    int largestPickedNum = 0;
    for (int j = ep; j >= sp; j--) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s > 5 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > min_laser_range || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > min_laser_range || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > min_laser_range) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < max_laser_range && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < max_laser_range && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < max_laser_range) {
        
        largestPickedNum++;
        /*
        sharp corner        --> .v = 2
        less sharp corner   --> .v = 1
        flate surface       --> .v = -1
        Less flate surface  --> .v = 0
        */
        // if (largestPickedNum <= 2) {
        if (largestPickedNum <= 10) {
          laserCloud->points[cloudSortInd[j]].v = 2;
          cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
          // std::cout << " cornerPointsSharp ind : " << cloudSortInd[j];
          // std::cout << "  x : " << laserCloud->points[cloudSortInd[j]].x;
          // std::cout << "  y : " <<laserCloud->points[cloudSortInd[j]].y;
          // std::cout << "  z : " <<laserCloud->points[cloudSortInd[j]].z << std::endl;

        // } else if (largestPickedNum <= 3) {
        } else if (largestPickedNum <= 10) {
          laserCloud->points[cloudSortInd[j]].v = 1;
          cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
          // std::cout << " cornerPointsLessSharp ind : " << cloudSortInd[j];
          // std::cout << "  x : " <<laserCloud->points[cloudSortInd[j]].x ;
          // std::cout << "  y : " <<laserCloud->points[cloudSortInd[j]].y ;
          // std::cout << "  z : " <<laserCloud->points[cloudSortInd[j]].z << std::endl;

        } else {
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
          ExcludedPoints->push_back(laserCloud->points[cloudSortInd[j] + k]);

        }
        for (int k = -1; k >= -5; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
          ExcludedPoints->push_back(laserCloud->points[cloudSortInd[j] + k]);

        }
      }
    }




/*Select 4 two points with very small curvature, and use the remaining points as points with 
relatively small curvature. Since less flat points are the most, perform voxel raster filtering
on each segment of less flat points, and the class implemented by PCL passes through
the input VoxelGridpoints Cloud data creates a 3D voxel grid (you can think of the voxel grid
as a collection of tiny 3D cubes), and then setInputCloudinput the point cloud, 
setLeafSizeset the voxel size, and use the volume The center of gravity of all points in 
the voxel is used to approximate other points in the voxel, so that all points in the voxel
are finally represented by a center of gravity, that is, the realization of filter:*/
   
   
//--------------------------------------------------------------
// divide the points in surfPointsFlat 
//----------------------------------------------------------------
    int smallestPickedNum = 0; 
    //Select the point where the curvature of each segment is very small and relatively small
    for (int j = sp; j <= ep; j++) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s < 0.5 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > min_laser_range || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > min_laser_range || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > min_laser_range) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < max_laser_range && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < max_laser_range && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < max_laser_range) {
 
        laserCloud->points[cloudSortInd[j]].v = -1; //-1 represents a point with a small curvature
        surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);
        *surfPoints = *surfPointsFlat;

        // std::cout << " surfPointsFlat ind : " << cloudSortInd[j];
        // std::cout << "  x : " <<laserCloud->points[cloudSortInd[j]].x;
        // std::cout << "  y : " <<laserCloud->points[cloudSortInd[j]].y;
        // std::cout << "  z : " <<laserCloud->points[cloudSortInd[j]].z << std::endl;

        smallestPickedNum++;
        if (smallestPickedNum >= 5) { //Select only the smallest four, and the remaining Label==0, all of which have relatively small curvature
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 30; k++) { //also prevent feature points from gathering
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k - 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k - 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.2) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
          ExcludedPoints->push_back(laserCloud->points[cloudSortInd[j] + k]);
        }

        for (int k = -1; k >= -30; k--) {
          float diffX = laserCloud->points[cloudSortInd[j] + k].x 
                      - laserCloud->points[cloudSortInd[j] + k + 1].x;
          float diffY = laserCloud->points[cloudSortInd[j] + k].y 
                      - laserCloud->points[cloudSortInd[j] + k + 1].y;
          float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
                      - laserCloud->points[cloudSortInd[j] + k + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.2) {
            break;
          }

          cloudNeighborPicked[cloudSortInd[j] + k] = 1;
          ExcludedPoints->push_back(laserCloud->points[cloudSortInd[j] + k]);

        }
      }
    }
  }


//--------------------------------------------------------------
// divide the points in surfPointsLessFlat 
//----------------------------------------------------------------
//All the remaining points (including the previously excluded points) are classified 
// into the less flat category in the plane point
  for (int i = 0; i < cloudSize; i++) {
    if (laserCloud->points[i].v == 0) {
      surfPointsLessFlat->push_back(laserCloud->points[i]);
    }
  }
//Because the less flat points have the most points, perform voxel raster filtering on the points of each segment less flat
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
  downSizeFilter.setInputCloud(surfPointsLessFlat);
  // downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilter.setLeafSize(0.2, .2, .2);
  downSizeFilter.filter(*surfPointsLessFlatDS);
   *surfPoints = *surfPointsLessFlatDS;


//less flat point summary
  *laserCloudExtreCur += *cornerPointsSharp;
  *laserCloudExtreCur += *surfPointsFlat;
  *laserCloudLessExtreCur += *cornerPointsLessSharp;
  *laserCloudLessExtreCur += *surfPointsLessFlatDS;


// =============================================
// extract the wall and ground points
// =============================================

   // Create separate clouds for ground points and wall planes
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr groundPoints(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr wallPoints(new pcl::PointCloud<pcl::PointXYZHSV>());
  // pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPoints(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsCombine(new pcl::PointCloud<pcl::PointXYZHSV>());

  // pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

   *surfPointsCombine += *surfPoints;
   
  
    // Set the Z-axis threshold to separate ground and wall points
    float z_threshold = -20.5; // Adjust this threshold as needed

    // Create a PassThrough filter to filter ground points
    pcl::PassThrough<pcl::PointXYZHSV> pass;
    pass.setInputCloud(surfPoints);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-std::numeric_limits<float>::max(), z_threshold);
    pass.filter(*groundPoints);
    
    sensor_msgs::PointCloud2 laserCloudGroundPoints;
    pcl::toROSMsg(*groundPoints, laserCloudGroundPoints);
    laserCloudGroundPoints.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudGroundPoints.header.frame_id = "world";
    pubLaserCloudGroundPointsPointer->publish(laserCloudGroundPoints); 


    // Create a PassThrough filter to filter wall points
    pass.setFilterLimits(z_threshold, std::numeric_limits<float>::max());
    pass.filter(*wallPoints);


    // Now groundPoints contains points with z < z_threshold
    // and wallPoints contains points with z >= z_threshold
 // Use Euclidean Cluster Extraction to segment the wall points into different clusters
    pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZHSV>());
    tree->setInputCloud(wallPoints);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> ec;
    ec.setClusterTolerance(2); // Adjust this value based on your data
    ec.setMinClusterSize(15);    // Minimum number of points in a cluster
    ec.setMaxClusterSize(2500000); // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(wallPoints);
    ec.extract(cluster_indices);
    
    // Create a vector to hold the clusters
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> wallClusters;

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZHSV>());
        for (const auto& index : indices.indices) {
            cluster->push_back((*wallPoints)[index]);
        }
        wallClusters.push_back(cluster);
    }

    // Sort clusters by size in descending order
    std::sort(wallClusters.begin(), wallClusters.end(), 
              [](const pcl::PointCloud<pcl::PointXYZHSV>::Ptr& a, const pcl::PointCloud<pcl::PointXYZHSV>::Ptr& b) {
                  return a->size() > b->size();
              });


 // Time of the current scan
    double timeScanCur = ros::Time::now().toSec();

    // // Select the largest 3 clusters and save them into wall1, wall2, and wall3
    // if (!wallClusters.empty()) {
    //     sensor_msgs::PointCloud2 laserCloudWall1Points;
    //     pcl::toROSMsg(*wallClusters[0], laserCloudWall1Points);
    //     laserCloudWall1Points.header.stamp = ros::Time().fromSec(timeScanCur);
    //     laserCloudWall1Points.header.frame_id = "world";
    //     pubLaserCloudWall1PointsPointer->publish(laserCloudWall1Points);
    // } else {
    //     publishEmptyCloud(*pubLaserCloudWall1PointsPointer, timeScanCur);
    // }

    // if (wallClusters.size() > 1) {
    //     sensor_msgs::PointCloud2 laserCloudWall2Points;
    //     pcl::toROSMsg(*wallClusters[1], laserCloudWall2Points);
    //     laserCloudWall2Points.header.stamp = ros::Time().fromSec(timeScanCur);
    //     laserCloudWall2Points.header.frame_id = "world";
    //     pubLaserCloudWall2PointsPointer->publish(laserCloudWall2Points);
    // } else {
    //     publishEmptyCloud(*pubLaserCloudWall2PointsPointer, timeScanCur);
    // }

    // if (wallClusters.size() > 2) {
    //     sensor_msgs::PointCloud2 laserCloudWall3Points;
    //     pcl::toROSMsg(*wallClusters[2], laserCloudWall3Points);
    //     laserCloudWall3Points.header.stamp = ros::Time().fromSec(timeScanCur);
    //     laserCloudWall3Points.header.frame_id = "world";
    //     pubLaserCloudWall3PointsPointer->publish(laserCloudWall3Points);
    // } else {
    //     publishEmptyCloud(*pubLaserCloudWall3PointsPointer, timeScanCur);
    // }





 

 
// =============================================
// Test

  for (int i = 0; i < 5; i++) {

    sensor_msgs::PointCloud2 laserExcludedPoints;
    pcl::toROSMsg(*ExcludedPoints, laserExcludedPoints);
    laserExcludedPoints.header.stamp = ros::Time().fromSec(timeScanCur);
    laserExcludedPoints.header.frame_id = "world";
    pubLaserExcludedPointsPointer->publish(laserExcludedPoints); 

    sensor_msgs::PointCloud2 laserCloudCornerPointsSharp;
    pcl::toROSMsg(*cornerPointsSharp, laserCloudCornerPointsSharp);
    laserCloudCornerPointsSharp.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudCornerPointsSharp.header.frame_id = "world";
    pubLaserCloudCornerPointsSharpPointer->publish(laserCloudCornerPointsSharp); 


    sensor_msgs::PointCloud2 laserCloudSurfPointsFlat;
    pcl::toROSMsg(*surfPointsFlat, laserCloudSurfPointsFlat);
    laserCloudSurfPointsFlat.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudSurfPointsFlat.header.frame_id = "world";
    pubLaserCloudSurfPointsFlatPointer->publish(laserCloudSurfPointsFlat); 


    sensor_msgs::PointCloud2 laserCloudSurfPointsLessFlat;
    pcl::toROSMsg(*surfPointsLessFlatDS, laserCloudSurfPointsLessFlat);
    laserCloudSurfPointsLessFlat.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudSurfPointsLessFlat.header.frame_id = "world";
    pubLaserCloudSurfPointsLessFlatPointer->publish(laserCloudSurfPointsLessFlat); 

    sensor_msgs::PointCloud2 laserCloudCornerPointsLessSharp;
    pcl::toROSMsg(*cornerPointsLessSharp, laserCloudCornerPointsLessSharp);
    laserCloudCornerPointsLessSharp.header.stamp = ros::Time().fromSec(timeScanCur);
    laserCloudCornerPointsLessSharp.header.frame_id = "world";
    pubLaserCloudCornerPointsLessSharpPointer->publish(laserCloudCornerPointsLessSharp); 

  }


  laserCloudIn->clear();
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
  surfPointsLessFlatDS->clear();
  ExcludedPoints->clear();
  groundPoints->clear();
wallPoints->clear();


// ==========================================================





  if (skipFrameCount >= skipFrameNum) {
    skipFrameCount = 0;
/*//publich IMU message, since the loop is at the end, so Cur represents the last point, 
that is, the Euler angle of the last point, the distortion displacement and the speed of a point 
cloud cycle increase*/
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

//The distortion displacement and velocity of the last point relative to the first point
    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

/*Publish ROS messages
All point clouds, larger curvature feature points, large curvature feature points, smaller curvature 
feature points, small curvature feature points, and IMU information after eliminating motion distortion 
are released here in turn. */
    sensor_msgs::PointCloud2 laserCloudExtreCur2;

    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudExtreCur2);
    laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
    // laserCloudExtreCur2.header.frame_id = "camera";
    laserCloudExtreCur2.header.frame_id = "world";
    
    pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2); ////publich eliminates all points after non-uniform motion distortion
    imuTrans->clear();

    pubLaserCloudLastPointer->publish(laserCloudLast2);






    // ROS_INFO ("%d %d", laserCloudLast2.width, laserCloudExtreCur2.width);
  }
  skipFrameCount++;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration_FeatExtrc");
  ros::NodeHandle nh;

    std::cout << " loam_based_features_estraction: " << std::endl;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/ld_lrs3611/pointcloud2", 2, laserCloudHandler);

  ros::Subscriber sub = nh.subscribe("/vins_estimator/odometry", 1000, odometry_callback);


  // ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> 
  //                          ("/imu/data1", 5, imuHandler);

  ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_extre_cur", 2);

  ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2> 
                                     ("/laser_cloud_last", 2);

  ros::Publisher pubLaserExcludedPoints = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/excluded_points", 2);
  ros::Publisher pubLaserCloudGroundPoints = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/ground_points", 2);
  
  ros::Publisher pubLaserCloudWall1Points = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/wall1_points", 2);
  ros::Publisher pubLaserCloudWall2Points = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/wall2_points", 2);
  ros::Publisher pubLaserCloudWall3Points = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/wall3_points", 2);
  ros::Publisher pubLaserCloudCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/CornerPointsSharp", 2);

  ros::Publisher pubLaserCloudSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/SurfPointsFlat", 2);

 ros::Publisher pubLaserCloudSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/SurfPointsLessFlat", 2);
  ros::Publisher pubLaserCloudCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2> 
                                      ("/CornerPointsLessSharp", 2);

  pubLaserCloudExtreCurPointer = &pubLaserCloudExtreCur;
  pubLaserCloudLastPointer = &pubLaserCloudLast;
  
  pubLaserExcludedPointsPointer = &pubLaserExcludedPoints;
  pubLaserCloudGroundPointsPointer = &pubLaserCloudGroundPoints;
pubLaserCloudWall1PointsPointer  = &pubLaserCloudWall1Points;
pubLaserCloudWall2PointsPointer  = &pubLaserCloudWall2Points;
pubLaserCloudWall3PointsPointer  = &pubLaserCloudWall3Points;


  pubLaserCloudCornerPointsSharpPointer = &pubLaserCloudCornerPointsSharp;
  pubLaserCloudSurfPointsFlatPointer = &pubLaserCloudSurfPointsFlat;
  pubLaserCloudSurfPointsLessFlatPointer = &pubLaserCloudSurfPointsLessFlat;
  pubLaserCloudCornerPointsLessSharpPointer = &pubLaserCloudCornerPointsLessSharp;
  ros::spin();


  return 0;
}

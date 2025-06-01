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

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double initTime;
double timeStart;
double timeLasted;
bool systemInited = false;

double timeScanCur = 0;
double timeScanLast = 0;

int laserRotDir = 1;
float laserAngleLast = 0;
float laserAngleCur = 0;

int skipFrameNum = 3;
int skipFrameCount = 0;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

sensor_msgs::PointCloud2 laserCloudExtreCur2;
sensor_msgs::PointCloud2 laserCloudLast2;

ros::Publisher* pubLaserCloudExtreCurPointer;
ros::Publisher* pubLaserCloudLastPointer;

int cloudSortInd[800];
int cloudNeighborPicked[800];

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

// Displacement and distortion calculation ShiftToStartIMU
// Calculate the distortion displacement caused by acceleration and deceleration relative to 
// the first point (distortion displacement delta_Tg in the global coordinate system)
//
void ShiftToStartIMU()
{
  // //Rotate around the y axis (-imuYawStart), ie Ry(yaw).inverse
  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

//Rotate around the x-axis (-imuPitchStart), ie Rx(pitch).inverse
  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

//Rotate around the z axis (-imuRollStart), ie Rz(pitch).inverse
  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

// Speed ​​distortion calculation VeloToStartIMU
// Calculate the velocity distortion (increment) of the points in the point cloud in the local coordinate system
// relative to the first starting point due to acceleration and deceleration
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

/*
Remove distortion TransformToStartIMU
Transfer the point cloud coordinates to the world coordinate system through the rotation relationship of imu (X imu − > X w X_{imu}->X_wx 
 and then transfer from the world coordinate system to the initial coordinate system of the current frame 
 through the angle at the beginning of the current frame (X w − > X scan X_w->X_{scan}x 
). Finally, add the displacement error calculated in ShiftToStartIM to complete the compensation.
*/
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

void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

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

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{

  //Discard the first 20 point cloud data:

  if (!systemInited) {
    initTime = laserCloudIn2->header.stamp.toSec();
    imuPointerFront = (imuPointerLast + 1) % imuQueLength;
    systemInited = true;
  }

// Get the current point cloud time ( the timestamp in ROS is generally: MsgPtr->header.stamp.toSec()
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

  bool newSweep = false;
  laserAngleLast = laserAngleCur;
  laserAngleCur = atan2(laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y, 
                        laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x);

  if (laserAngleLast > 0 && laserRotDir == 1 && laserAngleCur < laserAngleLast) {
    laserRotDir = -1;
    newSweep = true;
  } else if (laserAngleLast < 0 && laserRotDir == -1 && laserAngleCur > laserAngleLast) {
    laserRotDir = 1;
    newSweep = true;
  }

  if (newSweep) {
    timeStart = timeScanLast - initTime;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
    imuTrans->points[0].x = imuPitchStart;
    imuTrans->points[0].y = imuYawStart;
    imuTrans->points[0].z = imuRollStart;
    imuTrans->points[0].v = 10;

    imuTrans->points[1].x = imuPitchCur;
    imuTrans->points[1].y = imuYawCur;
    imuTrans->points[1].z = imuRollCur;
    imuTrans->points[1].v = 11;

    imuTrans->points[2].x = imuShiftFromStartXCur;
    imuTrans->points[2].y = imuShiftFromStartYCur;
    imuTrans->points[2].z = imuShiftFromStartZCur;
    imuTrans->points[2].v = 12;

    imuTrans->points[3].x = imuVeloFromStartXCur;
    imuTrans->points[3].y = imuVeloFromStartYCur;
    imuTrans->points[3].z = imuVeloFromStartZCur;
    imuTrans->points[3].v = 13;

    *laserCloudExtreCur += *laserCloudLessExtreCur;

//  int cloudSize = laserCloudIn->points.size();
    pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2);

// pcl::PointCloud<pcl::PointXYZHSV>::Ptr combined_pcd(new pcl::PointCloud<pcl::PointXYZHSV>());
// *combined_pcd = *laserCloudExtreCur + *imuTrans;
// std::cout << "laserCloudExtreCur: " << laserCloudExtreCur->points.size() <<  std::endl;
// std::cout << "imuTrans:           " << imuTrans->points.size() <<  std::endl;
// std::cout << "combined_pcd:" << combined_pcd->points.size() <<  std::endl;

    laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
    laserCloudLast2.header.frame_id = "camera";
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
  }

  imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
  imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
  imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;
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
// Calculate the curvature of each point cloud point
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
  }
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

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
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
    }
  }

//Extract feature points
// Divide all point clouds into four categories:
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

//Divide the curvature points of each scan into 6 equal parts
  int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 
                        6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
  int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 
                      5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

  for (int i = 0; i < 4; i++) {
    int sp = startPoints[i];
    int ep = endPoints[i];

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
    int largestPickedNum = 0;
    for (int j = ep; j >= sp; j--) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s > 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {
        
        largestPickedNum++;
        if (largestPickedNum <= 2) {
          laserCloud->points[cloudSortInd[j]].v = 2;
          cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
        } else if (largestPickedNum <= 20) {
          laserCloud->points[cloudSortInd[j]].v = 1;
          cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
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
   
   
    int smallestPickedNum = 0; 
    //Select the point where the curvature of each segment is very small and relatively small
    for (int j = sp; j <= ep; j++) {
      if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
          laserCloud->points[cloudSortInd[j]].s < 0.1 &&
          (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
          fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
          fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
          fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {
 
        laserCloud->points[cloudSortInd[j]].v = -1; //-1 represents a point with a small curvature
        surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

        smallestPickedNum++;
        if (smallestPickedNum >= 4) { //Select only the smallest four, and the remaining Label==0, all of which have relatively small curvature
          break;
        }

        cloudNeighborPicked[cloudSortInd[j]] = 1;
        for (int k = 1; k <= 5; k++) { //also prevent feature points from gathering
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
        }
      }
    }
  }

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
  downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter.filter(*surfPointsLessFlatDS);
//less flat point summary
  *laserCloudExtreCur += *cornerPointsSharp;
  *laserCloudExtreCur += *surfPointsFlat;
  *laserCloudLessExtreCur += *cornerPointsLessSharp;
  *laserCloudLessExtreCur += *surfPointsLessFlatDS;

  laserCloudIn->clear();
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
  surfPointsLessFlatDS->clear();

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
    laserCloudExtreCur2.header.frame_id = "camera";
    pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2); ////publich eliminates all points after non-uniform motion distortion
    imuTrans->clear();

    pubLaserCloudLastPointer->publish(laserCloudLast2);

    //ROS_INFO ("%d %d", laserCloudLast2.width, laserCloudExtreCur2.width);
  }
  skipFrameCount++;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/sync_scan_cloud_filtered", 2, laserCloudHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> 
                           ("/imu/data", 5, imuHandler);

  ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_extre_cur", 2);

  ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2> 
                                     ("/laser_cloud_last", 2);

  pubLaserCloudExtreCurPointer = &pubLaserCloudExtreCur;
  pubLaserCloudLastPointer = &pubLaserCloudLast;

  ros::spin();

  return 0;
}

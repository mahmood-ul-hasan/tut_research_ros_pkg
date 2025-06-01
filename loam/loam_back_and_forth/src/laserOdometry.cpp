/*The process comes to laserOdometry, whose main work is feature point matching and attitude calculation. 
Match the two received feature points (edge ​​points and plane points) using the scan-to-scan 
method to match two adjacent frames of point cloud data at time k and k+1, and 
calculate the point-line and point-plane distances . The attitude solution is to estimate 
the relative pose based on the matched feature point cloud. We will analyze them separately 
(Tucao: Why did the author put all the content in the mian function~).*/
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

 std::vector<int> minPointInd2_cornor_array; 
 std::vector<int> minPointInd2_surf_array; 
 std::vector<int> minPointInd3_surf_array; 
int counter = 0;
const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

// Timestamp information and message receiving flags are explained in the ROS release message 
bool systemInited = false;

double initTime;
double timeLasted;
double timeLastedRec;
double startTimeCur;
double startTimeLast;

double timeLaserCloudExtreCur = 0;
double timeLaserCloudLast = 0;

ros::Publisher pub_odm_prior_cloud, pub_odm_current_cloud ;

int nearestFeatureSearchSqDist = 30;

/*
initTime = laserCloudExtreCur2->header.stamp.toSec();
timeLaserCloudExtreCur = laserCloudExtreCur2->header.stamp.toSec();
timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();

timeLasted = timeLaserCloudExtreCur - initTime;
timeLastedRec = timeLasted;

startTimeCur = timeLaserCloudLast - initTime;
startTimeLast = startTimeCur;


initTime = time when first laserCloudExtractCur recieved
timeLasted = difference b/w curent time and initTime
*/

bool newLaserCloudExtreCur = false;
bool newLaserCloudLast = false;

// Accept point cloud information from scanRegistration:

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreOri(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreUnsel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreProj(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

// relative transfer from the previous frame and the first frame

float transform[6] = {0};  //The state transition amount of the current frame relative to the previous frame, in the local frame
float transformRec[6] = {0};
float transformSum[6] = {0}; //The state transition amount of the current frame relative to the first frame, in the global frame

// Distortion record
float imuRollStartCur = 0, imuPitchStartCur = 0, imuYawStartCur = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

float imuRollStartLast = 0, imuPitchStartLast = 0, imuYawStartLast = 0;
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuShiftFromStartXLast = 0, imuShiftFromStartYLast = 0, imuShiftFromStartZLast = 0;
float imuVeloFromStartXLast = 0, imuVeloFromStartYLast = 0, imuVeloFromStartZLast = 0;

bool imuInited = true;

// Calculate whether the pose is legal and whether the iteration condition is satisfied
void TransformReset()
{
for (int i = 0; i < 6; i++) {
  transformRec[i] = transform[i];
  transform[i] = 0;
}

transformRec[3] -= imuVeloFromStartXLast * (startTimeCur - startTimeLast);
transformRec[4] -= imuVeloFromStartYLast * (startTimeCur - startTimeLast);
transformRec[5] -= imuVeloFromStartZLast * (startTimeCur - startTimeLast);
}


/*=====================================================
V. Distortion removal function
The two functions involved in this part are used 
a) TransformToStart()
b) TransformToEnd()
=======================================================
*/
/*First of all, we need to clarify the difference between the distortion removal of imu in the
last chapter, or how the distortion removal based on imu in the last chapter serves for the
distortion removal here. The first thing that needs to be clarified is that the object 
transportation process, certainly can not be uniform motion all the time, but then, the 
removal of motion aberrations here is based on the assumption of uniform motion model, which
there is the error caused by the model, if we add the imu data, then the error brought by 
the model can be compensated off before building the model. Recalling the previous chapter 
on removing motion distortion caused by acceleration and deceleration, we will find that the
difference obtained by subtracting the first point cloud point from the current point cloud
  point, and then subtracting the displacement of uniform motion is compensated to the 
  current point cloud point. At this time, if we assume the uniform motion model here, 
  the error due to the model can be eliminated.

----------------------------------------------------------------------------------------
1. Remove the uniform motion distortion relative to the start point TransformToStart
---------------------------------------------------------------------------------------

Recall the calculation of the point intensity information of the point cloud in the previous
chapter: s c a n I D + s c a n P e r i o d ∗ r e l T i m e 
scanID+scanPeriod*relTimescanID+scanPeriod∗relTime , and because the period of the radar 
is 0.1s, the relative time of the point cloud is: 10 ∗ r e l T i m e 10* relTime10∗relTime 
, and the point cloud equivalent to the stationary point cloud at the starting point can be
obtained by linear interpolation using this value.

// The points in the current point cloud are relative to the first point to remove the 
distortion caused by the uniform motion, the effect is equivalent to getting the point cloud
at the start of the point cloud scanning stationary position*/
void TransformToStart(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
// interpolation factor calculation, relative time per point in the cloud / point cloud period 10
float s = (pi->h - startTime) / (endTime - startTime);  

// linear interpolation: multiply the corresponding rotation translation factor by the 
//relative position of each point in the point cloud
float rx = s * transform[0];
float ry = s * transform[1];
float rz = s * transform[2];
float tx = s * transform[3];
float ty = s * transform[4];
float tz = s * transform[5];

//Use the relative attitude after interpolation to convert the current point to the first point. 
// It should be noted that the corresponding Euler angles must take negative values:
//  //rotate (-rz) around z-axis after translation
float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
float z1 = (pi->z - tz);

//rotate around x-axis (-rx)
float x2 = x1;
float y2 = cos(rx) * y1 + sin(rx) * z1;
float z2 = -sin(rx) * y1 + cos(rx) * z1;

//rotate around the y-axis (-ry)
po->x = cos(ry) * x2 - sin(ry) * z2;
po->y = y2;
po->z = sin(ry) * x2 + cos(ry) * z2;
po->h = pi->h;
po->s = pi->s;
po->v = pi->v;
}

/* -----------------------------------------------------------------------
Remove the relative end point uniform motion distortion TransformToEnd
----------------------------------------------------------------------- */

/*The main function of this function is to convert the laser point of the
current frame to the end of the current frame and remove the imu deviation. The
function first interpolates the pose matrix according to the relative time of the
current laser point. The function first converts the laser point to the initial point
coordinate system of the current frame according to the relative time of the current
laser point, and then converts to the end point coordinate system. Then compensate
the imu position error. Since the imu position error is the end point of the
current frame in the initial time coordinate system, it first rotates to the world
coordinate system, and then rotates to the end time imu coordinate system. First find the
relative transformation between the current point and the first point, and convert to
the coordinates of the first point (similar to TransformToStart):   
*/
void TransformToEnd(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
float s = (pi->h - startTime) / (endTime - startTime);

float rx = s * transform[0];
float ry = s * transform[1];
float rz = s * transform[2];
float tx = s * transform[3];
float ty = s * transform[4];
float tz = s * transform[5];

/*Use the relative attitude after interpolation to convert the current point to
the coordinate system where the first point is located. It should be noted that
the corresponding Euler angles must take negative values: 
*/
float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
float z1 = (pi->z - tz);

float x2 = x1;
float y2 = cos(rx) * y1 + sin(rx) * z1;
float z2 = -sin(rx) * y1 + cos(rx) * z1;

float x3 = cos(ry) * x2 - sin(ry) * z2;
float y3 = y2;
float z3 = sin(ry) * x2 + cos(ry) * z2;

//Convert points from the coordinate system of the first point to the coordinate system of the last point
rx = transform[0];
ry = transform[1];
rz = transform[2];
tx = transform[3];
ty = transform[4];
tz = transform[5];

float x4 = cos(ry) * x3 + sin(ry) * z3;
float y4 = y3;
float z4 = -sin(ry) * x3 + cos(ry) * z3;

float x5 = x4;
float y5 = cos(rx) * y4 - sin(rx) * z4;
float z5 = sin(rx) * y4 + cos(rx) * z4;

float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
float z6 = z5 + tz;


// Sorry~~~, I have been confused by the coordinate transformation here~
float x7 = cos(imuRollStartLast) * (x6 - imuShiftFromStartXLast) 
          - sin(imuRollStartLast) * (y6 - imuShiftFromStartYLast);
float y7 = sin(imuRollStartLast) * (x6 - imuShiftFromStartXLast) 
          + cos(imuRollStartLast) * (y6 - imuShiftFromStartYLast);
float z7 = z6 - imuShiftFromStartZLast;

float x8 = x7;
float y8 = cos(imuPitchStartLast) * y7 - sin(imuPitchStartLast) * z7;
float z8 = sin(imuPitchStartLast) * y7 + cos(imuPitchStartLast) * z7;

float x9 = cos(imuYawStartLast) * x8 + sin(imuYawStartLast) * z8;
float y9 = y8;
float z9 = -sin(imuYawStartLast) * x8 + cos(imuYawStartLast) * z8;

float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
float y10 = y9;
float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

float x11 = x10;
float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
po->z = z11;
po->h = pi->h;
po->s = pi->s;
po->v = pi->v;
}


/*=====================================================
VI. Attitude transformation function
The two functions involved in this part are used in the coordinate transformation of the eighth part.
a) void AccumulateRotation()
b) Void PluginIMURotation()
=======================================================
*/

/*--------------------------------------------------------
1. Accumulate rotation amount AccumulateRotation
-------------------------------------------------------
After obtaining the relative rotation of the two backward frames, it is
necessary to calculate the rotation transformation of the current frame relative to the
first frame, that is, the accumulation of the rotation amount. Calculation formula:R
c u r s t a r t = R l a s t s t a r t ∗ ( R l a s t c u r ) − 1
R_{cur}^{start}=R_{last}^{start}*(R_{last}^{cur })^{-1}R  c u r s t a r t ​  =R  l a s t s t a r t ​  ∗( _  l a s t c u r ​  )
− 1   have to be aware of is:R curlast = R y R x R z R_{cur}^{last}=R_yR_xR_zR
c u r l a s t ​  =R  the y ​  R  x ​  R  z ​  ,andR laststart = R z R x R y
R_{last}^{start}=R_zR_xR_yR  l a s t s t a r t ​  =R  z ​  R  x ​  R  the y ​   Finally find out
(-sin(rx))=cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx) - cos(cx)*cos(lx) *sin(cz)*sin(ly) and in the
program is (-sin(rx))= cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz )*sin(lx) -
cos(lx)*cos(ly)*sin(cx); (srx=(-sin(rx)) in the program) it can be found that the difference between
the two formulas is lx, ly, lz negative sign, so the accumulateRotation() function
passes in the negative value of transform[0] 
*/
void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                      float &ox, float &oy, float &oz)
{
float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
ox = -asin(srx);

float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
              + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
              - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
              + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
              - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

/*2. Correct the current point Euler angle PluginIMURotation 
------------------------------------------------------------
In fact, the principle of correction is not complicated, using the formula( R c u r s t a r t ) ′ = R
e n d R s t a r t − 1 R c u r s t a r t
(R_{cur}^{start})'=R_{end}R_{start}^{-1}R_{cur}^{start}( _  c u r s t a r t ​  )  '  =R  e n d ​  R  s t a r t − 1 ​  R  c u r s t a r
t ​  that's it, pay attentionR end R_{end}R  e n d ​  andR start − 1
R_{start}^{-1}R  s t a r t − 1 ​  Both are zxy rotations. The main reason is that the
derivation is annoying, so I won't show it here.  
*/
void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                      float alx, float aly, float alz, float &acx, float &acy, float &acz)
{
float sbcx = sin(bcx);
float cbcx = cos(bcx);
float sbcy = sin(bcy);
float cbcy = cos(bcy);
float sbcz = sin(bcz);
float cbcz = cos(bcz);

float sblx = sin(blx);
float cblx = cos(blx);
float sbly = sin(bly);
float cbly = cos(bly);
float sblz = sin(blz);
float cblz = cos(blz);

float salx = sin(alx);
float calx = cos(alx);
float saly = sin(aly);
float caly = cos(aly);
float salz = sin(alz);
float calz = cos(alz);

float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
          - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
          - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
          - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
          - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
acx = -asin(srx);

float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
              - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
              - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
              - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
              + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
              - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
              - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
              - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
              + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
              - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
              - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
              + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
              - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
              + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
              + calx*cblx*salz*sblz);
float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
              - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
              + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
              + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
              + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
              - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
              - calx*calz*cblx*sblz);
acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}


/*
initTime = laserCloudExtreCur2->header.stamp.toSec();
timeLaserCloudExtreCur = laserCloudExtreCur2->header.stamp.toSec();
timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();

timeLasted = timeLaserCloudExtreCur - initTime;
timeLastedRec = timeLasted;

startTimeCur = timeLaserCloudLast - initTime;
startTimeLast = startTimeCur;


initTime = time when first laserCloudExtractCur recieved
timeLasted = difference b/w curent time and initTime
*/

void laserCloudExtreCurHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudExtreCur2)
{
      std::cout << "laserCloudExtreCurHandler recieved data"  <<std::endl;
      
       pub_odm_current_cloud.publish(*laserCloudExtreCur2);


if (!systemInited) {
  initTime = laserCloudExtreCur2->header.stamp.toSec();
  systemInited = true;
}
timeLaserCloudExtreCur = laserCloudExtreCur2->header.stamp.toSec();
timeLasted = timeLaserCloudExtreCur - initTime;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur3(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::fromROSMsg(*laserCloudExtreCur2, *laserCloudExtreCur3);
int laserCloudExtreCur3Size = laserCloudExtreCur3->points.size();

laserCloudExtreCur->clear();
for (int i = 0; i < laserCloudExtreCur3Size; i++) {



  if (fabs(laserCloudExtreCur3->points[i].v - 10) < 0.005) {
    imuPitchStartCur = laserCloudExtreCur3->points[i].x;
    imuYawStartCur = laserCloudExtreCur3->points[i].y;
    imuRollStartCur = laserCloudExtreCur3->points[i].z;
  } else if (fabs(laserCloudExtreCur3->points[i].v - 11) < 0.005) {
    imuPitchCur = laserCloudExtreCur3->points[i].x;
    imuYawCur = laserCloudExtreCur3->points[i].y;
    imuRollCur = laserCloudExtreCur3->points[i].z;
  } else if (fabs(laserCloudExtreCur3->points[i].v - 12) < 0.005) {
    imuShiftFromStartXCur = laserCloudExtreCur3->points[i].x;
    imuShiftFromStartYCur = laserCloudExtreCur3->points[i].y;
    imuShiftFromStartZCur = laserCloudExtreCur3->points[i].z;
  } else if (fabs(laserCloudExtreCur3->points[i].v - 13) < 0.005) {
    imuVeloFromStartXCur = laserCloudExtreCur3->points[i].x;
    imuVeloFromStartYCur = laserCloudExtreCur3->points[i].y;
    imuVeloFromStartZCur = laserCloudExtreCur3->points[i].z;
  } else {
    laserCloudExtreCur->push_back(laserCloudExtreCur3->points[i]);
  }
}
laserCloudExtreCur3->clear();

if (!imuInited) {
  transformSum[0] += imuPitchStartCur;
  //transformSum[1] += imuYawStartCur;
  transformSum[2] += imuRollStartCur;

  imuInited = true;
}

// if (timeLasted > 4.0) {
if (timeLasted > 150.0) {
  newLaserCloudExtreCur = true;
}
}


void laserCloudLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudLast2)
{
    std::cout << "laserCloudLastHandler recieved data"  <<std::endl;
      pub_odm_prior_cloud.publish(*laserCloudLast2);

    
  // Check whether all information has been received and require time synchronization and set all flags to false after acceptance:
if (laserCloudLast2->header.stamp.toSec() > timeLaserCloudLast + 0.005) {
  timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();
  startTimeLast = startTimeCur;
  startTimeCur = timeLaserCloudLast - initTime;


// Save the edge points and plane points of the current frame as the data of the previous moment
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudPointer = laserCloudCornerLLast;
  laserCloudCornerLLast = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudPointer;

  laserCloudPointer = laserCloudSurfLLast;
  laserCloudSurfLLast = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudPointer;

  laserCloudLast->clear();
  pcl::fromROSMsg(*laserCloudLast2, *laserCloudLast);
  int laserCloudLastSize = laserCloudLast->points.size();
  // std::cout << "laserCloudLastSize" << laserCloudLastSize << std::endl;
  laserCloudExtreLast->clear();
  laserCloudCornerLast->clear();
  laserCloudSurfLast->clear();

/*
sharp corner        --> .v = 2
less sharp corner   --> .v = 1
flate surface       --> .v = -1
Less flate surface  --> .v = 0
*/
  for (int i = 0; i < laserCloudLastSize; i++) {

    

    if (fabs(laserCloudLast->points[i].v - 2) < 0.005 || fabs(laserCloudLast->points[i].v + 1) < 0.005) {
      laserCloudExtreLast->push_back(laserCloudLast->points[i]);
    } 
    if (fabs(laserCloudLast->points[i].v - 2) < 0.005 || fabs(laserCloudLast->points[i].v - 1) < 0.005) {
      laserCloudCornerLast->push_back(laserCloudLast->points[i]);
    } 
    if (fabs(laserCloudLast->points[i].v) < 0.005 || fabs(laserCloudLast->points[i].v + 1) < 0.005) {
      laserCloudSurfLast->push_back(laserCloudLast->points[i]);
    }
    if (fabs(laserCloudLast->points[i].v - 10) < 0.005) {
      imuPitchStartLast = laserCloudLast->points[i].x;
      imuYawStartLast = laserCloudLast->points[i].y;
      imuRollStartLast = laserCloudLast->points[i].z;
    }
    if (fabs(laserCloudLast->points[i].v - 11) < 0.005) {
      imuPitchLast = laserCloudLast->points[i].x;
      imuYawLast = laserCloudLast->points[i].y;
      imuRollLast = laserCloudLast->points[i].z;
    }
    if (fabs(laserCloudLast->points[i].v - 12) < 0.005) {
      imuShiftFromStartXLast = laserCloudLast->points[i].x;
      imuShiftFromStartYLast = laserCloudLast->points[i].y;
      imuShiftFromStartZLast = laserCloudLast->points[i].z;
    }
    if (fabs(laserCloudLast->points[i].v - 13) < 0.005) {
      imuVeloFromStartXLast = laserCloudLast->points[i].x;
      imuVeloFromStartYLast = laserCloudLast->points[i].y;
      imuVeloFromStartZLast = laserCloudLast->points[i].z;
    }
  }

  pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreePointer = kdtreeCornerLLast;
  kdtreeCornerLLast = kdtreeCornerLast;
  kdtreeCornerLast = kdtreePointer;
  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);

  kdtreePointer = kdtreeSurfLLast;
  kdtreeSurfLLast = kdtreeSurfLast;
  kdtreeSurfLast = kdtreePointer;
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  // if (timeLasted > 4.0) {
  if (timeLasted > 150.0) {
    newLaserCloudLast = true;
  }
  
}
}



int main(int argc, char** argv)
{
ros::init(argc, argv, "laserOdometry");
ros::NodeHandle nh;


pub_odm_prior_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/odm_prior_cloud", 2);
 pub_odm_current_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/odm_current_cloud", 2);

ros::Subscriber subLaserCloudExtreCur = nh.subscribe<sensor_msgs::PointCloud2> 
                                        ("/laser_cloud_extre_cur", 2, laserCloudExtreCurHandler);
ros::Subscriber subLaserCloudLast = nh.subscribe<sensor_msgs::PointCloud2> 
                                    ("/laser_cloud_last", 2, laserCloudLastHandler);
ros::Publisher pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);



ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 1);
ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 1);
ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 1);
ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 1);
ros::Publisher pub5 = nh.advertise<sensor_msgs::PointCloud2> ("/pc5", 1);
ros::Publisher pub6 = nh.advertise<sensor_msgs::PointCloud2> ("/pc6", 1);

ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
nav_msgs::Odometry laserOdometry;
laserOdometry.header.frame_id = "camera_init";
laserOdometry.child_frame_id = "camera";

tf::TransformBroadcaster tfBroadcaster;
tf::StampedTransform laserOdometryTrans;
laserOdometryTrans.frame_id_ = "camera_init";
laserOdometryTrans.child_frame_id_ = "camera";


  // Create a publisher for the Pose message
// Create a publisher for the Euler angles and translation
  ros::Publisher posePub = nh.advertise<geometry_msgs::PointStamped>("pose", 10);
  


std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;
std::vector<int> pointSelInd;

pcl::PointXYZHSV extreOri, extreSel, extreProj, tripod1, tripod2, tripod3, coeff;

/*Create the count and cycle frequency (100hz) sent to the mapping task. Both ros:
:spinOnce() and ros::spin() are ROS message callback processing functions. The d
ifference is that the latter will not return after calling, that is Your main pr
ogram will not be executed here, but the former can continue to execute the foll
owing program after calling*/
bool status = ros::ok();


while (status) {

  std::cout << "while() starts " << counter <<std::endl;

  ros::spinOnce();

  bool sweepEnd = false;
  bool newLaserPoints = false;
  bool sufficientPoints = false;
  double startTime, endTime;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr extrePointsPtr, laserCloudCornerPtr, laserCloudSurfPtr;
  pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerPtr, kdtreeSurfPtr;
  ros::Time time_start1;
  ros::Time time_end1;
 ros::Time time_start2;
  ros::Time time_end2;
double time_iterate =0;

/*
initTime = laserCloudExtreCur2->header.stamp.toSec();
timeLaserCloudExtreCur = laserCloudExtreCur2->header.stamp.toSec();
timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();

timeLasted = timeLaserCloudExtreCur - initTime;
timeLastedRec = timeLasted;

startTimeCur = timeLaserCloudLast - initTime;
startTimeLast = startTimeCur;


initTime = time when first laserCloudExtractCur recieved
timeLasted = difference b/w curent time and initTime
*/


  if (newLaserCloudExtreCur && newLaserCloudLast) {
std::cout << "if (newLaserCloudExtreCur && newLaserCloudLast)"  <<std::endl;

    startTime = startTimeLast;
    endTime = startTimeCur;

    extrePointsPtr = laserCloudExtreLast;
    laserCloudCornerPtr = laserCloudCornerLLast;
    laserCloudSurfPtr = laserCloudSurfLLast;
    kdtreeCornerPtr = kdtreeCornerLLast;
    kdtreeSurfPtr = kdtreeSurfLLast;

    laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
    laserOdometryTrans.stamp_ = ros::Time().fromSec(timeLaserCloudLast);

    sweepEnd = true;
    newLaserPoints = true;

// Iterative optimization is performed when the number of edge points is greater than 10 and the number of plane points is greater than 100
    if (laserCloudSurfLLast->points.size() >= 100) {
      sufficientPoints = true;
    }

  }
  else if (newLaserCloudExtreCur) {
std::cout << "   else if (newLaserCloudExtreCur)"  <<std::endl;

    startTime = startTimeCur;
    endTime = timeLasted;

    extrePointsPtr = laserCloudExtreCur;
    laserCloudCornerPtr = laserCloudCornerLast;
    laserCloudSurfPtr = laserCloudSurfLast;
    kdtreeCornerPtr = kdtreeCornerLast;
    kdtreeSurfPtr = kdtreeSurfLast;

    laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    laserOdometryTrans.stamp_ = ros::Time().fromSec(timeLaserCloudExtreCur);

    float s = (timeLasted - timeLastedRec) / (startTimeCur - startTimeLast);
    for (int i = 0; i < 6; i++) {
      transform[i] += s * transformRec[i];        
    }
    timeLastedRec = timeLasted;

    newLaserPoints = true;

    if (laserCloudSurfLast->points.size() >= 100) {
      sufficientPoints = true;
    }
  }


// -------------------------------------------------
  if (newLaserPoints && sufficientPoints) {

std::cout << "if (newLaserPoints && sufficientPoints)"  <<std::endl;

    newLaserCloudExtreCur = false;
    newLaserCloudLast = false;

    int extrePointNum = extrePointsPtr->points.size();
    int laserCloudCornerNum = laserCloudCornerPtr->points.size();
    int laserCloudSurfNum = laserCloudSurfPtr->points.size();

    float st = 1;
    if (!sweepEnd) {
      st = (timeLasted - startTime) / (startTimeCur - startTimeLast);
    }
    // int iterNum = st * 50;
    int iterNum = st * 2;

    int pointSelSkipNum = 2;

time_start1 = ros::Time::now();
std::cout << " time1_started: "  << " " << iterNum  << " extrePointsPtr " << extrePointNum<<std::endl;

/*
=============================================================
itrations for features matching
-============================================================
*/
    for (int iterCount = 0; iterCount < iterNum; iterCount++) {
      if(iterCount <1)
      std::cout << "for (int iterCount = 0; iterCount < iterNum; iterCount++) " << iterCount << " " << iterNum  << "extrePointsPtr " << extrePointNum <<std::endl;
      
time_start2 = ros::Time::now();



      laserCloudExtreOri->clear();
      laserCloudExtreSel->clear();
      laserCloudExtreUnsel->clear();
      laserCloudExtreProj->clear();
      laserCloudSel->clear();
      coeffSel->clear();

// Find the nearest point every five iterations, first find the nearest neighbor point in the KD tree:


      bool isPointSel = false;
      if (iterCount % (pointSelSkipNum + 1) == 0) {
        isPointSel = true;
        pointSelInd.clear();
      }
      

/*
=============================================================
Plane and Cornor points (feature) processing
-============================================================
*/
      for (int i = 0; i < extrePointNum; i++) {
        
        // std::cout << "Plane and Cornor points (feature) processing" << i << std::endl;
        extreOri = extrePointsPtr->points[i];
        TransformToStart(&extreOri, &extreSel, startTime, endTime);

        if (isPointSel) {
          pointSelInd.push_back(-1);
          pointSelInd.push_back(-1);
          pointSelInd.push_back(-1);
        }

/*
=============================================================
Plane point (surface feature) processing
-============================================================
Similarly, firstly, the basic idea is to extract the points with small curvature,
first find the nearest neighbor point in the point with small curvature in the previous
frame, and then find another point in the adjacent scan of the scan where the nearest
neighbor point is located. A point with the closest distance, in addition, it is necessary
to find another point with the smallest distance (or the second smallest point) in
the same frame of the nearest neighbor point, and three points can form a plane. The
specific analysis is as follows:  First project each surface feature point (plane point)
to the coordinate system of the starting point of the fram 

  
sharp corner        --> .v = 2
less sharp corner   --> .v = 1
flate surface       --> .v = -1
Less flate surface  --> .v = 0

==============================================================
*/



        if (fabs(extreOri.v + 1) < 0.05) {


    // std::cout << "Plane point (surface feature) processing " <<std::endl;


// Find the nearest neighbor every fifth iteration in the kd tree
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (isPointSel) {
// performing a nearest neighbor search using a kd-tree (kdtreeSurfPtr) to find the nearest point in the extreSel point cloud. It searches for the 1 nearest 
// neighbor (nearestKSearch) and stores the index of the nearest point in pointSearchInd and the squared distance to the nearest point in pointSearchSqDis
            kdtreeSurfPtr->nearestKSearch(extreSel, 1, pointSearchInd, pointSearchSqDis);
            
            // minPointInd2_cornor_array.push_back(pointSearchSqDis[0]);


  
            if (pointSearchSqDis[0] > nearestFeatureSearchSqDist) {
              continue;
            }

            closestPointInd = pointSearchInd[0];
            float closestPointTime = laserCloudSurfPtr->points[closestPointInd].h;

// Find the remaining two points except the nearest neighbor point in the direction of scanID increase
            // float pointSqDis, minPointSqDis2 = 1, minPointSqDis3 = 1;
            float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
            for (int j = closestPointInd + 1; j < laserCloudSurfNum; j++) {
              if (laserCloudSurfPtr->points[j].h > closestPointTime + 0.07) {
                break;
              }

              pointSqDis = (laserCloudSurfPtr->points[j].x - extreSel.x) * 
                            (laserCloudSurfPtr->points[j].x - extreSel.x) + 
                            (laserCloudSurfPtr->points[j].y - extreSel.y) * 
                            (laserCloudSurfPtr->points[j].y - extreSel.y) + 
                            (laserCloudSurfPtr->points[j].z - extreSel.z) * 
                            (laserCloudSurfPtr->points[j].z - extreSel.z);


              if (laserCloudSurfPtr->points[j].h < closestPointTime + 0.005) {
                  if (pointSqDis < minPointSqDis2) {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                  }
              } else {
                  if (pointSqDis < minPointSqDis3) {
                    minPointSqDis3 = pointSqDis;
                    minPointInd3 = j;
                  }
              }
            }

//Find the remaining two points except the nearest neighbor point in the direction of scanID decreas             
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (laserCloudSurfPtr->points[j].h < closestPointTime - 0.07) {
                break;
              }

              pointSqDis = (laserCloudSurfPtr->points[j].x - extreSel.x) * 
                            (laserCloudSurfPtr->points[j].x - extreSel.x) + 
                            (laserCloudSurfPtr->points[j].y - extreSel.y) * 
                            (laserCloudSurfPtr->points[j].y - extreSel.y) + 
                            (laserCloudSurfPtr->points[j].z - extreSel.z) * 
                            (laserCloudSurfPtr->points[j].z - extreSel.z);

              if (laserCloudSurfPtr->points[j].h > closestPointTime - 0.005) {
                  if (pointSqDis < minPointSqDis2) {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                  }
              } else {
                  if (pointSqDis < minPointSqDis3) {
                    minPointSqDis3 = pointSqDis;
                    minPointInd3 = j;
                  }
              }
            }
              // minPointInd2_surf_array.push_back(minPointSqDis2);
              // minPointInd3_surf_array.push_back(minPointSqDis3);

          } 
          
          
          else {
            if (pointSelInd[3 * i] >= 0) {
              closestPointInd = pointSelInd[3 * i];
              minPointInd2 = pointSelInd[3 * i + 1];
              minPointInd3 = pointSelInd[3 * i + 2];

              float distX = extreSel.x - laserCloudSurfPtr->points[closestPointInd].x;
              float distY = extreSel.y - laserCloudSurfPtr->points[closestPointInd].y;
              float distZ = extreSel.z - laserCloudSurfPtr->points[closestPointInd].z;
              if (distX * distX + distY * distY + distZ * distZ > 1.0) {
                continue;
              }
            } else {
              continue;
            }
              // minPointInd2_surf_array.push_back(0);
              // minPointInd3_surf_array.push_back(0);


          }

  
  /*-----------------------------------
  The point-to-plane distance 
  -----------------------------------
  After finding the corresponding point of the feature point, the work is to
  calculate the distance. The point-to-plane distance formula is consistent with the paper:
  compared  with the line feature, there is also one more weight calculation and plane
  normal vector (that is, the direction vector of the point to the plane line) The
  decomposition in the direction of each axis (basically consistent) is also used in the
  calculation of Jacobian. 
  */



          if (minPointInd2 >= 0 && minPointInd3 >= 0) {
        
          // std::cout << "The point-to-plane distance " <<std::endl;

            tripod1 = laserCloudSurfPtr->points[closestPointInd];
            tripod2 = laserCloudSurfPtr->points[minPointInd2];
            tripod3 = laserCloudSurfPtr->points[minPointInd3];

            float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
                      - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
            float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
                      - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
            float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
                      - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
            float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            float pd2 = pa * extreSel.x + pb * extreSel.y + pc * extreSel.z + pd;

            extreProj = extreSel;
            extreProj.x -= pa * pd2;
            extreProj.y -= pb * pd2;
            extreProj.z -= pc * pd2;

            float s = 1;
            if (iterCount >= 30) {
              s = 1 - 8 * fabs(pd2) / sqrt(sqrt(extreSel.x * extreSel.x
                + extreSel.y * extreSel.y + extreSel.z * extreSel.z));
            }

            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
            coeff.h = s * pd2;

            if (s > 0.2 || iterNum < 80)
             {
              laserCloudExtreOri->push_back(extreOri);
              laserCloudExtreSel->push_back(extreSel);
              laserCloudExtreProj->push_back(extreProj);
              laserCloudSel->push_back(tripod1);
              laserCloudSel->push_back(tripod2);
              laserCloudSel->push_back(tripod3);
              coeffSel->push_back(coeff);

              if (isPointSel) {
                pointSelInd[3 * i] = closestPointInd;
                pointSelInd[3 * i + 1] = minPointInd2;
                pointSelInd[3 * i + 2] = minPointInd3;
              }
            } 
            else 
            {
              laserCloudExtreUnsel->push_back(extreSel);
            }
          }
       

        } 
        



/*
=============================================================
Edge point (line feature) processing
-============================================================
it should be noted that the basic idea is to extract the points with large
curvature, first find the nearest neighbor point in the point with large curvature in the
previous frame, and then find another closest neighbor point in the adjacent scan of the
scan where the nearest neighbor point is located. point, the specific analysis is as
follows 

    
sharp corner        --> .v = 2
less sharp corner   --> .v = 1
flate surface       --> .v = -1
Less flate surface  --> .v = 0
        
==============================================================
*/
// if(i<1){time_start2 = ros::Time::now();
//         std::cout << i << " time2_started: "   <<std::endl;}
  
  
  
  else if (fabs(extreOri.v - 2) < 0.05) {
        



// std::cout << "Edge point (line feature) processing" <<std::endl;

// Find the nearest point every five iterations, first find the nearest neighbor point in the KD tree
          int closestPointInd = -1, minPointInd2 = -1;
          if (isPointSel) {
            kdtreeCornerPtr->nearestKSearch(extreSel, 1, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] > nearestFeatureSearchSqDist) {
              continue;
            }

/*Find the point with the smallest distance between the adjacent line and the
target point (confusing behavior, it’s not that you know the scanID of the nearest
neighbor, it’s so troublesome)
*/
            closestPointInd = pointSearchInd[0];
            float closestPointTime = laserCloudCornerPtr->points[closestPointInd].h;

/*initial threshold value of 5 meters, can roughly filter out scanID adjacent,
but the actual line is not adjacent to the value
*/
            // float pointSqDis, minPointSqDis2 = 1;
            float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
 //find the point with the smallest sum of squares of the closest distance to the target point

            for (int j = closestPointInd + 1; j < laserCloudCornerNum; j++) {
              if (laserCloudCornerPtr->points[j].h > closestPointTime + 0.07) {
                break;
              }

              pointSqDis = (laserCloudCornerPtr->points[j].x - extreSel.x) * 
                            (laserCloudCornerPtr->points[j].x - extreSel.x) + 
                            (laserCloudCornerPtr->points[j].y - extreSel.y) * 
                            (laserCloudCornerPtr->points[j].y - extreSel.y) + 
                            (laserCloudCornerPtr->points[j].z - extreSel.z) * 
                            (laserCloudCornerPtr->points[j].z - extreSel.z);
//make sure two points are not on the same scan (adjacency lookup should be possible with scanID == closestPointScan +/- 1 to do it
              if (laserCloudCornerPtr->points[j].h > closestPointTime + 0.005) {
//closer distance, to be less than the initial value of 5 meters
                  if (pointSqDis < minPointSqDis2) {
                 //Update the minimum distance with the point order
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                  }
              }
            }
      
      //similarly //Find in the direction of decreasing scanID

            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (laserCloudCornerPtr->points[j].h < closestPointTime - 0.07) {
                break;
              }

              pointSqDis = (laserCloudCornerPtr->points[j].x - extreSel.x) * 
                            (laserCloudCornerPtr->points[j].x - extreSel.x) + 
                            (laserCloudCornerPtr->points[j].y - extreSel.y) * 
                            (laserCloudCornerPtr->points[j].y - extreSel.y) + 
                            (laserCloudCornerPtr->points[j].z - extreSel.z) * 
                            (laserCloudCornerPtr->points[j].z - extreSel.z);

              if (laserCloudCornerPtr->points[j].h < closestPointTime - 0.005) {
                  if (pointSqDis < minPointSqDis2) {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                  }
              }
            }
          } 
          else {
            if (pointSelInd[3 * i] >= 0) {
              closestPointInd = pointSelInd[3 * i];
              minPointInd2 = pointSelInd[3 * i + 1];

              float distX = extreSel.x - laserCloudCornerPtr->points[closestPointInd].x;
              float distY = extreSel.y - laserCloudCornerPtr->points[closestPointInd].y;
              float distZ = extreSel.z - laserCloudCornerPtr->points[closestPointInd].z;
              if (distX * distX + distY * distY + distZ * distZ > 1.0) {
                continue;
              }
            } else {
              continue;
            }
          }


/*-----------------------------------
  The point-to-line distance
  -----------------------------------
The next work is relatively simple, and the calculation of the distance is
consistent with the paper (if you are not familiar with it, you can read the paper LOAM:
Lidar Odometry and Mapping in Real-time .) The formula is as follows:  
It is worth noting that in the code five times After the iteration, there is an additional weight
calculation. The larger the distance, the smaller the weight, and the smaller the distance,
the larger the weight. The obtained weight range is less than or equal to 1. There is
also a noteworthy point. The code not only calculates the distance from the point to
the line, but also obtains the direction vector from the point to the line, and the
decomposition of the direction vector on each axis, which will be used in the Jacobian solution*/
              //  minPointInd2_cornor_array.push_back(minPointSqDis2);

          if (minPointInd2 >= 0) {

            // std::cout << "The point-to-line distance " <<std::endl;

            tripod1 = laserCloudCornerPtr->points[closestPointInd];
            tripod2 = laserCloudCornerPtr->points[minPointInd2];

            float x0 = extreSel.x;
            float y0 = extreSel.y;
            float z0 = extreSel.z;
            float x1 = tripod1.x;
            float y1 = tripod1.y;
            float z1 = tripod1.z;
            float x2 = tripod2.x;
            float y2 = tripod2.y;
            float z2 = tripod2.z;

            float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                      * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                      + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                      * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                      + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                      * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

            float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

            float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                      + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

            float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                      - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

            float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                      + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

            float ld2 = a012 / l12;

            extreProj = extreSel;
            extreProj.x -= la * ld2;
            extreProj.y -= lb * ld2;
            extreProj.z -= lc * ld2;

            float s = 2 * (1 - 8 * fabs(ld2));

            coeff.x = s * la;
            coeff.y = s * lb;
            coeff.z = s * lc;
            coeff.h = s * ld2;

            if (s > 0.4)
             {
              laserCloudExtreOri->push_back(extreOri);
              laserCloudExtreSel->push_back(extreSel);
              laserCloudExtreProj->push_back(extreProj);
              laserCloudSel->push_back(tripod1);
              laserCloudSel->push_back(tripod2);
              coeffSel->push_back(coeff);

              if (isPointSel) {
                pointSelInd[3 * i] = closestPointInd;
                pointSelInd[3 * i + 1] = minPointInd2;
              }
            } 
            else {
              laserCloudExtreUnsel->push_back(extreSel);
            }
          }

        // if(i >= extrePointNum-1){
        // time_end2 = ros::Time::now();
        // ros::Duration time_elapsed2 = time_end2 - time_start2;
        // std::cout << i << " time_elapsed2 cornor features: " << time_elapsed2.toSec()  <<std::endl;}
        }
      }


//=======================================================
//
//
//   For Loop for features ends here
//=======================================================

        
// There are at least 10 feature points that meet the requirements, and the number of feature 
// matches is too small to discard this frame dat
      int extrePointSelNum = laserCloudExtreOri->points.size();
      if (extrePointSelNum < 10) {
        continue;
      }


/*=====================================================
VIII. Pose Estimation
The two functions involved in this part are used in the coordinate transformation of the eighth part.
1. Calculation of Jacobian For the loss
2. Matrix Solving

=======================================================

After the feature points are matched, the pose estimation is
started, and the problem is converted into a least squares optimization problem in loam,
and the LM method is used for iterative solution, so the focus of optimization is on
the derivative of the Jacobian matrix.  

1. Calculation of Jacobian
-----------------------------------
 For the loss function (loss is the distance calculated by the feature point, the code uses B matrix 
representation): l o s s = F ( X ~ k + 1 , T k + 1 ) loss=F(\tilde{X}_{k+1},T_{k+1}) l o s s=F ( 
x ~    k + 1 ​  ,T  k + 1 ​  )  

Its Jacobian matrix (used in the code A Matrix representation) is: 

J = ∂ F ∂ T k + 1 = ∂ F ∂ X ~ k + 1 ∂ X ~ k + 1 ∂ T k + 1 J=\frac{\partial
F}{\partial T_{k+1}}=\frac{\partial F}{\partial \tilde{X}_{k+1}}\frac{\partial
\tilde{X}_{k+1}}{\partial T_{k+1}} J=  ∂T _  k + 1 ​   ∂ F ​  =  ∂  x ~    k + 1 ​   ∂ F ​    ∂T _  k + 1 ​
∂  x ~    k + 1 ​   ​    in∂ F ∂ X ~ k + 1 \frac{\partial F}{\partial
\tilde{X}_{k+1}}  ∂  x ~    k + 1 ​   ∂ F ​  It has been completed. If it is a line feature, it is
the unit vector from the point to the line direction, and if it is a line feature, it
is the unit vector from the point to the plane direction. about∂ X ~ k + 1 ∂ T k + 1
\frac{\partial \tilde{X}_{k+1}}{\partial T_{k+1}}  ∂T _  k + 1 ​   ∂  x ~    k + 1 ​   ​  j can
be derived separately for translation and rotation, so I won’t repeat them here. So
in the end the solution to the problem becomes the solution: 
*/

// Define individual matrices
      cv::Mat matA(extrePointSelNum, 6, CV_32F, cv::Scalar::all(0)); // Its Jacobian matrix (A Matrix representation) is: 
      cv::Mat matAt(6, extrePointSelNum, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matB(extrePointSelNum, 1, CV_32F, cv::Scalar::all(0)); //  For the loss function (loss is the distance calculated by the feature point, the code uses B matrix 
      cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

// Solve the Jacobian matrix A and loss vector B for each feature point, and assemble them into a large solution matrix
      for (int i = 0; i < extrePointSelNum; i++) {
        extreOri = laserCloudExtreOri->points[i];
        coeff = coeffSel->points[i];

        float s = (extreOri.h - startTime) / (endTime - startTime);

        float srx = sin(s * transform[0]);
        float crx = cos(s * transform[0]);
        float sry = sin(s * transform[1]);
        float cry = cos(s * transform[1]);
        float srz = sin(s * transform[2]);
        float crz = cos(s * transform[2]);
        float tx = s * transform[3];
        float ty = s * transform[4];
        float tz = s * transform[5];

        float arx = (-s*crx*sry*srz*extreOri.x + s*crx*crz*sry*extreOri.y + s*srx*sry*extreOri.z 
                  + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                  + (s*srx*srz*extreOri.x - s*crz*srx*extreOri.y + s*crx*extreOri.z
                  + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                  + (s*crx*cry*srz*extreOri.x - s*crx*cry*crz*extreOri.y - s*cry*srx*extreOri.z
                  + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

        float ary = ((-s*crz*sry - s*cry*srx*srz)*extreOri.x 
                  + (s*cry*crz*srx - s*sry*srz)*extreOri.y - s*crx*cry*extreOri.z 
                  + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx) 
                  + s*tz*crx*cry) * coeff.x
                  + ((s*cry*crz - s*srx*sry*srz)*extreOri.x 
                  + (s*cry*srz + s*crz*srx*sry)*extreOri.y - s*crx*sry*extreOri.z
                  + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry) 
                  - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

        float arz = ((-s*cry*srz - s*crz*srx*sry)*extreOri.x + (s*cry*crz - s*srx*sry*srz)*extreOri.y
                  + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                  + (-s*crx*crz*extreOri.x - s*crx*srz*extreOri.y
                  + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                  + ((s*cry*crz*srx - s*sry*srz)*extreOri.x + (s*crz*sry + s*cry*srx*srz)*extreOri.y
                  + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

        float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y 
                  - s*(crz*sry + cry*srx*srz) * coeff.z;

        float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y 
                  - s*(sry*srz - cry*crz*srx) * coeff.z;

        float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

        float d2 = coeff.h;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arz;
        matA.at<float>(i, 3) = atx;
        matA.at<float>(i, 4) = aty;
        matA.at<float>(i, 5) = atz;
        // matB.at<float>(i, 0) = -0.015 * st * d2;
        matB.at<float>(i, 0) = -0.5 * st * d2;
        minPointInd2_surf_array.push_back(-0.5*st * d2);
        minPointInd3_surf_array.push_back(st);
        minPointInd2_cornor_array.push_back(d2);
        // std::cout << " " << -0.05 * st * d2  ;



      }


// std::cout << "Matrix Solving" <<std::endl;
/*-----------------------------------
2. Matrix Solving
-----------------------------------
Use the OpenCV function to solve, and use QR decomposition to accelerate
*/
      cv::transpose(matA, matAt);
      matAtA = matAt * matA; //+ 0.1 * cv::Mat::eye(6, 6, CV_32F);
      matAtB = matAt * matB;
      cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
      //cv::solve(matA, matB, matX, cv::DECOMP_SVD);

/*-----------------------------------
4. Pose update
-----------------------------------
Pose updates using iteratively computed deltas
*/
      if (fabs(matX.at<float>(0, 0)) < 0.005 &&
          fabs(matX.at<float>(1, 0)) < 0.005 &&
          fabs(matX.at<float>(2, 0)) < 0.005 &&
          fabs(matX.at<float>(3, 0)) < 0.01 &&
          fabs(matX.at<float>(4, 0)) < 0.01 &&
          fabs(matX.at<float>(5, 0)) < 0.01) {

        //transform[0] += 0.7 * matX.at<float>(0, 0);
        //transform[1] += 0.7 * matX.at<float>(1, 0);
        //transform[2] += 0.7 * matX.at<float>(2, 0);
        transform[0] += 0.1 * matX.at<float>(0, 0);
        transform[1] += 0.1 * matX.at<float>(1, 0);
        transform[2] += 0.1 * matX.at<float>(2, 0);
        transform[3] += matX.at<float>(3, 0);
        transform[4] += matX.at<float>(4, 0);
        transform[5] += matX.at<float>(5, 0);


time_end2 = ros::Time::now();
ros::Duration time_elapsed2 = time_end2 - time_start2;
time_iterate += time_elapsed2.toSec();
std::cout << time_elapsed2.toSec()<<  " -- " ;      

      } else {
        ROS_INFO ("Odometry update out of bound");
      }
    }

time_end1 = ros::Time::now();
ros::Duration time_elapsed1 = time_end1 - time_start1;
std::cout << std::endl;

std::cout << " time_elapsed1 for features: " << time_elapsed1.toSec() << " = " <<  time_iterate <<std::endl;      
std::cout <<  "transform ";
for (int i = 0; i < 6; i++) { 
std::cout << transform[i] << " ";}
std::cout << std::endl;


//=======================================================
//
//
//   For Loop for multiple features ends here
//=======================================================

    sensor_msgs::PointCloud2 pc12;
    // pcl::toROSMsg(*laserCloudCornerPtr, pc12);
    pcl::toROSMsg(*laserCloudExtreOri, pc12);
    pc12.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    pc12.header.frame_id = "camera";
    pub1.publish(pc12);
    sensor_msgs::PointCloud2 pc22;
    pcl::toROSMsg(*laserCloudSurfPtr, pc22);
    pc22.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    pc22.header.frame_id = "camera";
    pub2.publish(pc22);
    sensor_msgs::PointCloud2 pc32;
    pcl::toROSMsg(*laserCloudExtreSel, pc32);
    pc32.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    pc32.header.frame_id = "camera";
    pub3.publish(pc32);
    sensor_msgs::PointCloud2 pc42;
    pcl::toROSMsg(*laserCloudExtreUnsel, pc42);
    pc42.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    pc42.header.frame_id = "camera";
    pub4.publish(pc42);
    sensor_msgs::PointCloud2 pc52;
    pcl::toROSMsg(*laserCloudExtreProj, pc52);
    pc52.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    pc52.header.frame_id = "camera";
    pub5.publish(pc52);
    sensor_msgs::PointCloud2 pc62;
    pcl::toROSMsg(*laserCloudSel, pc62);
    pc62.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
    pc62.header.frame_id = "camera";
    pub6.publish(pc62);
  }


// ===================================================================
//-------------------------------------------------------------
//=====================================================================

/*-----------------------------------
5. Coordinate transformation
-----------------------------------*/
std::cout << "Coordinate transformation" <<std::endl;

  if (newLaserPoints) {
    float rx, ry, rz, tx, ty, tz;
    if (sweepEnd) {
      AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                          -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);

      float x1 = cos(rz) * (transform[3] - imuShiftFromStartXLast) 
                - sin(rz) * (transform[4] - imuShiftFromStartYLast);
      float y1 = sin(rz) * (transform[3] - imuShiftFromStartXLast) 
                + cos(rz) * (transform[4] - imuShiftFromStartYLast);
      float z1 = transform[5] * 1.05 - imuShiftFromStartZLast;

      float x2 = x1;
      float y2 = cos(rx) * y1 - sin(rx) * z1;
      float z2 = sin(rx) * y1 + cos(rx) * z1;

      tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
      ty = transformSum[4] - y2;
      tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

      PluginIMURotation(rx, ry, rz, imuPitchStartLast, imuYawStartLast, imuRollStartLast, 
                        imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

      int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      for (int i = 0; i < laserCloudCornerLastNum; i++) {
        TransformToEnd(&laserCloudCornerLast->points[i], &laserCloudCornerLast->points[i], 
                        startTimeLast, startTimeCur);
      }

      int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
      for (int i = 0; i < laserCloudSurfLastNum; i++) {
        TransformToEnd(&laserCloudSurfLast->points[i], &laserCloudSurfLast->points[i], 
                        startTimeLast, startTimeCur);
      }

      TransformReset();

      transformSum[0] = rx;
      transformSum[1] = ry;
      transformSum[2] = rz;
      transformSum[3] = tx;
      transformSum[4] = ty;
      transformSum[5] = tz;

      sensor_msgs::PointCloud2 laserCloudLast2;
      pcl::toROSMsg(*laserCloudCornerLast + *laserCloudSurfLast, laserCloudLast2);
      laserCloudLast2.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      laserCloudLast2.header.frame_id = "camera";
      pubLaserCloudLast2.publish(laserCloudLast2);

    } else {

// std::cout <<  "transform1 ";
// for (int i = 0; i < 6; i++) { 
// std::cout << transform[i] << " ";}
// std::cout << std::endl;

//  std::cout <<  "transformSum ";
// for (int i = 0; i < 6; i++) { 
// std::cout << transformSum[i] << " ";}
// std::cout << std::endl;

      AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                          -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);
    
      float x1 = cos(rz) * (transform[3] - imuShiftFromStartXCur) 
                - sin(rz) * (transform[4] - imuShiftFromStartYCur);
      float y1 = sin(rz) * (transform[3] - imuShiftFromStartXCur) 
                + cos(rz) * (transform[4] - imuShiftFromStartYCur);
      float z1 = transform[5] * 1.05 - imuShiftFromStartZCur;

      float x2 = x1;
      float y2 = cos(rx) * y1 - sin(rx) * z1;
      float z2 = sin(rx) * y1 + cos(rx) * z1;


      tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
      ty = transformSum[4] - y2;
      tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

      PluginIMURotation(rx, ry, rz, imuPitchStartCur, imuYawStartCur, imuRollStartCur, 
                        imuPitchCur, imuYawCur, imuRollCur, rx, ry, rz);
    }

       
//Euler angles converted to quaternions
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);
// Publish odometer information, one for mapping and one for rviz

    
    laserOdometry.pose.pose.orientation.x = -geoQuat.y;
    laserOdometry.pose.pose.orientation.y = -geoQuat.z;
    laserOdometry.pose.pose.orientation.z = geoQuat.x;
    laserOdometry.pose.pose.orientation.w = geoQuat.w;
    laserOdometry.pose.pose.position.x = tx;
    laserOdometry.pose.pose.position.y = ty;
    laserOdometry.pose.pose.position.z = tz;
    pubLaserOdometry.publish(laserOdometry);





    laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
    tfBroadcaster.sendTransform(laserOdometryTrans);

    //ROS_INFO ("%f %f %f %f %f %f", transformSum[0], transformSum[1], transformSum[2], 
    //                               transformSum[3], transformSum[4], transformSum[5]);


   
    
    // Publish the Euler angles and translation
    geometry_msgs::PointStamped eulerMsg;
     eulerMsg.header.frame_id = "camera_init";
    eulerMsg.header.stamp = ros::Time::now();

    // Convert quaternion to Euler angles
    tf2::Quaternion quat(-geoQuat.y,-geoQuat.z, geoQuat.x, geoQuat.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    // Convert angles to degrees
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    eulerMsg.point.x = roll;
    eulerMsg.point.y = pitch;
    eulerMsg.point.z = yaw;

    
std::cout << "Publish odometer information" <<std::endl;
std::cout <<   eulerMsg.point <<std::endl;
std::cout <<     laserOdometry.pose.pose.position <<std::endl;


    // Publish the Euler angles and translation
    posePub.publish(eulerMsg);

        counter = counter +1;


if (counter > 50){
  counter = 0;
plt::figure();
plt::suptitle("LaserAngle Cur, Last, and RotDir"); // add a title
plt::subplot(3, 1, 1); plt::plot(minPointInd2_surf_array, {{"label", "minPointInd2_surf_array"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
plt::subplot(3, 1, 2); plt::plot(minPointInd3_surf_array, {{"label", "minPointInd3_surf_array"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
plt::subplot(3, 1, 3); plt::plot(minPointInd2_cornor_array, {{"label", "minPointInd2_cornor_array"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");

plt::figure();
plt::suptitle("LaserAngle Cur, Last, and RotDir"); // add a title
plt::subplot(1, 1, 1); plt::plot(minPointInd2_surf_array, {{"label", "minPointInd2_surf_array"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
plt::subplot(1, 1, 1); plt::plot(minPointInd3_surf_array, {{"label", "minPointInd3_surf_array"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");
plt::subplot(1, 1, 1); plt::plot(minPointInd2_cornor_array, {{"label", "minPointInd2_cornor_array"}}); plt::legend(); plt::xlabel("number of points"); plt::ylabel("index of points"); plt::grid("true");


plt::show();
}


  }

  status = ros::ok();
  cv::waitKey(10);
}

return 0;
}

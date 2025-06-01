/****************************************************************************

Conversion from a quaternion to roll, pitch and yaw.

Nodes:
subscribed /rotation_quaternion (message of type geometry_msgs::Quaternion)
published /rpy_angles (message oftype geometry_msgs::Vector3.h)

****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/String.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>


// #include "LinearMath/btMatrix3x3.h"

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;
double pi = 3.14159;
void static_transform_publisher(std::string header, std::string child, double x, double y, double z, double r, double p, double h);


// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.



void imu_upper_link_callback(const geometry_msgs::QuaternionStamped msg)
{
    
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.quaternion, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  // this Vector is then published:
  // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x*180/pi, rpy.y*180/pi, rpy.z*180/pi);

  static tf2_ros::TransformBroadcaster br;


  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "lower_link";
  t.child_frame_id = "upper_link";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, rpy.z);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  br.sendTransform(t);

  std::cout << "\n upper_link angle is equal to " << rpy.z*180/pi; 


}




void imu_boom_link_callback(const geometry_msgs::QuaternionStamped msg)
{
    
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.quaternion, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // this Vector is then published:
    rpy_publisher.publish(rpy);
    // ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x*180/pi, rpy.y*180/pi, rpy.z*180/pi);

    static tf2_ros::TransformBroadcaster br;


    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = "upper_link";
    t.child_frame_id = "boom_link";
    t.transform.translation.x = 1.1;
    t.transform.translation.y = 0.184;
    t.transform.translation.z = 0.866;
    tf2::Quaternion q;
    q.setRPY(0, -(90*pi/180 + rpy.y), 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    br.sendTransform(t);

    

    static_transform_publisher("world", "base_link", 0,0,0, 0, 0,0);
    static_transform_publisher("base_link", "lower_link",0,0, 1.334, 0, 0, 0);
    static_transform_publisher("boom_link", "laser", 20.392 ,0.63, 0.82, -90*pi/180, 6*pi/180, 180*pi/180); 
    std::cout << "  boom angle is equal to " << rpy.y*180/pi <<"=" << (90*pi/180 + rpy.y)*180/pi; 


}


void static_transform_publisher(std::string header, std::string child, double x, double y, double z, double r, double p, double h)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped t;

  t.header.stamp = ros::Time::now();
  t.header.frame_id = header;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  tf2::Quaternion quat;
  quat.setRPY(r, p, h);
  t.transform.rotation.x = quat.x();
  t.transform.rotation.y = quat.y();
  t.transform.rotation.z = quat.z();
  t.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(t);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf2_broadcaster_kobelco");
    ros::NodeHandle n;

    ros::Publisher rpy_publisher = n.advertise<geometry_msgs::Vector3>("/rpy_imu_boom_link1", 1000);
    ros::Subscriber imu_boom_subscriber = n.subscribe("/imu/quaternion", 1000, imu_boom_link_callback);
    ros::Subscriber imu_upper_subscriber = n.subscribe("/imu/quaternion", 1000, imu_upper_link_callback);
   
    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for tf2_broadcaster_kobelco");
    ros::spin();
    return 0;
}





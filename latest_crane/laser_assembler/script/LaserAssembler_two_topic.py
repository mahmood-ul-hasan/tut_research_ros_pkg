#!/usr/bin/env python3

import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

def call_assemble_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        assemble_scans = rospy.ServiceProxy(service_name, AssembleScans2)
        response = assemble_scans(rospy.Time(0, 0), rospy.get_rostime())
        return response.cloud
    except rospy.ServiceException as e:
        rospy.logerr("Service call to %s failed: %s" % (service_name, e))
        return None

def main():
    rospy.init_node("assemble_scans_to_cloud")

    pub1 = rospy.Publisher("/laser_pointcloud_assembler", PointCloud2, queue_size=1)
    pub2 = rospy.Publisher("/laser_pointcloud_assembler_line", PointCloud2, queue_size=1)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("Attempting to call /laser_assembler_1/assemble_scans2 service")
        cloud1 = call_assemble_service('/laser_assembler_1/assemble_scans2')

        rospy.loginfo("Attempting to call /laser_assembler_2/assemble_scans2 service")
        cloud2 = call_assemble_service('/laser_assembler_2/assemble_scans2')

        if cloud1:
            rospy.loginfo("Publishing cloud from /laser_assembler_1 with %d points", len(cloud1.data))
            pub1.publish(cloud1)
        else:
            rospy.logwarn("No cloud received from /laser_assembler_1")

        if cloud2:
            rospy.loginfo("Publishing cloud from /laser_assembler_2 with %d points", len(cloud2.data))
            pub2.publish(cloud2)
        else:
            rospy.logwarn("No cloud received from /laser_assembler_2")

        rate.sleep()

if __name__ == '__main__':
    main()

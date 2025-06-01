#!/usr/bin/env python
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2


rospy.init_node("test_client")
rospy.wait_for_service("assemble_scans")
pub = rospy.Publisher ("/pointcloud_assembler", PointCloud2, queue_size=1)
r = rospy.Rate(1)

while not rospy.is_shutdown():
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        print "Got cloud with %u points" % len(resp.cloud.points)
        #pub.publish (resp.cloud)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    r.sleep()

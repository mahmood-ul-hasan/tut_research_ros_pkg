#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tf
import numpy as np
import math as m
import matplotlib.pyplot as plt

def Polar2Cartesian(r, alpha):
    return np.transpose(np.array([np.cos(alpha) * r, np.sin(alpha) * r]))

def Cartesian2Polar(x, y):
    r = np.sqrt(x ** 2 + y ** 2)
    phi = np.arctan2(y, x)
    return r, phi

def MakeMarker(points):
    marker = Marker()
    marker.header.frame_id = "laser"
    marker.type = marker.LINE_STRIP
    marker.action = marker.MODIFY

    # marker scale
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01

    # marker color
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0

    # marker orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    for i in range(points.shape[0]):
        p = Point()
        p.x, p.y, p.z = points[i, 0], points[i, 1], 0
        marker.points.append(p)
    return marker

def GetPolar(X, Y):
    X = X - np.mean(X)
    Y = Y - np.mean(Y)
    k, n = np.polyfit(X, Y, 1)
    alpha = m.atan(-1 / k)
    ro = n / (m.sin(alpha) - k * m.cos(alpha))
    return ro, alpha

def CheckPolar(ro, alpha):
    if ro < 0:
        alpha = alpha + m.pi
        if alpha > m.pi:
            alpha = alpha - 2 * m.pi
        ro = -ro
    return ro, alpha

def getDistance(P, Ps, Pe):
    if np.all(np.equal(Ps, Pe)):
        return np.linalg.norm(P - Ps)
    return np.divide(np.abs(np.linalg.norm(np.cross(Pe - Ps, Ps - P))), np.linalg.norm(Pe - Ps))

def GetMostDistant(P):
    dmax = 0
    index = -1
    for i in range(1, P.shape[0]):
        d = getDistance(P[i, :], P[0, :], P[-1, :])
        if d > dmax:
            index = i
            dmax = d
    return dmax, index

def SplitAndMerge(P, threshold):
    d, ind = GetMostDistant(P)
    if d > threshold:
        P1 = SplitAndMerge(P[:ind + 1, :], threshold)
        P2 = SplitAndMerge(P[ind:, :], threshold)
        points = np.vstack((P1[:-1, :], P2))
    else:
        points = np.vstack((P[0, :], P[-1, :]))
    return points

def callback(data):
    global pub_rviz, threshold, isPrint
    ranges = np.asarray(data.ranges)
    alpha = np.linspace(data.angle_min, data.angle_max, len(ranges))  # Ensure alpha has the same length as ranges

    # Debugging print statements
    print(f"Length of ranges: {len(ranges)}")
    print(f"Length of alpha: {len(alpha)}")

    ranges[ranges == np.inf] = 250  # Clip the distance to 3.5m
    P = Polar2Cartesian(ranges, alpha)
    points = SplitAndMerge(P, threshold)
    for i in range(points.shape[0] - 1):
        ro, alpha = GetPolar(points[i:i + 2, 0], points[i:i + 2, 1])
        ro, alpha = CheckPolar(ro, alpha)
        if isPrint:
            print("ro = {}, alpha = {}".format(ro, alpha))
    if isPrint:
        print("-------------------------------------")
    marker = MakeMarker(points)
    pub_rviz.publish(marker)


if __name__ == '__main__':
    rospy.init_node('main')
    threshold = 0.05
    isPrint = False
    vel = Twist()

    pub_rviz = rospy.Publisher('visualization_msgs/Marker', Marker, queue_size=0)
    rospy.Subscriber("/ld_lrs3611/scan_filtered", LaserScan, callback)
    rospy.spin()

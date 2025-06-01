#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf.transformations import euler_from_matrix
import tf.transformations as tft
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, quaternion_from_matrix
k = 1


def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    rot_matrix = euler_matrix(r, p, h, 'rzyx')

    # quat_static = quaternion_from_euler(r, p, h)

    quat_static = quaternion_from_matrix(rot_matrix)
    # quat_static = rotation_from_euler(r, p, h)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    # print(t)
    br.sendTransform(t)


def matrx4x4_to_eular_pos(T, given_axes):
    # Extract the rotation component of the transformation matrix
    R = T[:3, :3]
    # Extract the translation component of the transformation matrix
    al, be, ga = euler_from_matrix(R, axes=given_axes)
    pos = T[:3, 3]
    angle = [al * 180 / np.pi, be * 180 / np.pi, ga * 180 / np.pi]
    return pos, angle, np.round(R, decimals=5)


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_by_imu')

    # Example transformation matrix
    given_axes = 'sxyz'
    T = np.array([
        [0.99878347, -0.01471939, 0.0470629, 0.00080373],
        [0.01347784, 0.99955556, 0.02659012, -0.00115897],
        [-0.04743338, -0.02592347, 0.99853796, -0.01696154],
        [0.0, 0.0, 0.0, 1.0]
    ])

    T = np.array([
        [0.00257233, 0.99994471, 0.01019585, 0.01055654],
        [-0.99917387, 0.00215655, 0.04058231, 0.00676656],
        [0.04055808, -0.01029182, 0.99912418, 0.02091211],
        [0., 0., 0., 1.]
    ])

    pos, angle, R = matrx4x4_to_eular_pos(T, given_axes)
    print("\n matrx4x4_to_pos: ", pos, " \n matrx4x4_to_angle: ", angle, "\n matrx4x4_to_angle: \n", R)


# ==================================
# Convert matrix to euler
# ==================================

    R1 = np.array([
        [-1.0000000e+00, -1.2246468e-16, 0.0000000e+00],
        [1.2246468e-16, -1.0000000e+00, -0.0000000e+00],
        [0.0000000e+00, 0.0000000e+00, 1.0000000e+00]
    ])
    # print("euler angle1 :\n", al, " ", be, " ", ga)

    R1 = np.array([
        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, -1]
    ])

    R1 = np.array([
        [0.285172, 0.934445, -0.213282],
        [-0.542008, -0.0263073, -0.839961],
        [-0.790509, 0.355134, 0.498975],
    ])

    al, be, ga = euler_from_matrix(R1, axes='sxyz')
    print("euler angle:\n", al * 180 / np.pi, " ", be * 180 / np.pi, " ", ga * 180 / np.pi)


# ==================================
# Convert euler to matrix
# matrix to eular
# ==================================

    # px = 70.14 / 1000
    # py = -7.38 / 1000
    # pz = -1.4 / 1000
    # d = [px, py, pz]
    # r = 0 * np.pi / 180
    # p = 180 * np.pi / 180
    # y = 90 * np.pi / 180

    px = 3.5 / 100
    py = 0 / 1000
    pz = -1.4 / 100
    d = [px, py, pz]
    r = 0 * np.pi / 180
    p = 0 * np.pi / 180
    y = 0 * np.pi / 180
    rot_matrix = euler_matrix(r, p, y, 'rxyz')
    rot_matrix = rot_matrix[0:3, 0:3]
    rot_matrix = np.round(rot_matrix, decimals=5)
    print(" rot_matrix \n", rot_matrix)

    r = 90 * np.pi / 180
    p = 0 * np.pi / 180
    y = -90 * np.pi / 180
    rot_matrix = euler_matrix(r, p, y, 'rxyz')
    rot_matrix = rot_matrix[0:3, 0:3]
    rot_matrix = np.round(rot_matrix, decimals=5)
    print(" rot_matrix \n", rot_matrix)

    # Calculate the inverse of the matrix
    rot_matrix_inv = np.linalg.inv(rot_matrix)
    rot_matrix_inv = np.round(rot_matrix_inv, decimals=5)

    print(" rot_matrix_inv \n", rot_matrix_inv)

    al, be, ga = euler_from_matrix(rot_matrix, axes='rxyz')
    # print("euler angle:\n", al * 180 / np.pi, " ", be * 180 / np.pi, " ", ga * 180 / np.pi)

    print("d \n", d)
    # for i in range(0, 10000):
    i = 1
    while i < 6:
        static_transform_publisher("camera", "imu", px, py, pz, r, p, y)
        # static_transform_publisher("imu", "camera", 100 / 1000, 0 / 1000, 0 / 1000, 0 * np.pi / 180, 0, 0 * np.pi / 180)

#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Imu
from tf.transformations import euler_from_quaternion
from scipy.optimize import least_squares
from math import sin, cos

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Return the ZYX (yaw->pitch->roll) rotation matrix."""
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                   [sin(yaw),  cos(yaw), 0],
                   [0, 0, 1]])
    Ry = np.array([[cos(pitch), 0, sin(pitch)],
                   [0, 1, 0],
                   [-sin(pitch), 0, cos(pitch)]])
    Rx = np.array([[1, 0, 0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll),  cos(roll)]])
    return Rz @ Ry @ Rx  # Correct for ZYX convention

def transform_point(params, alpha, beta, point):
    """
    Apply the calibration transformation chain to a LiDAR point.
    
    Transformation chain:
      World = T_world_trans @ T_yaw_rot @ T_pitch_trans @ T_pitch_rot @ T_lidar @ P_lidar
      
    where:
      - T_lidar: LiDAR-to-Pitch Motor transformation (rotation and translation)
      - T_pitch_rot: Pitch rotation by beta
      - T_pitch_trans: Translation from yaw motor to pitch motor ([p_x, p_y, p_z_pitch])
      - T_yaw_rot: Yaw rotation by alpha
      - T_world_trans: Translation from yaw motor to world ([0, 0, p_z_world])
    
    Parameters:
      params: list of 10 calibration parameters
      alpha: measured yaw angle from IMU
      beta: measured pitch angle from IMU
      point: (x, y, z) LiDAR point
    Returns:
      world_point: transformed 3D point in world frame
    """
    # Unpack calibration parameters
    p_z_world = params[0]              # Yaw motor height (vertical offset)
    p_x, p_y, p_z_pitch = params[1:4]    # Pitch motor translation relative to yaw motor
    t_x, t_y, t_z = params[4:7]          # LiDAR translation relative to pitch motor
    roll, pitch, yaw = params[7:10]       # LiDAR rotation (ZYX order)

    # 1. LiDAR to Pitch Motor transformation
    R_lidar = euler_to_rotation_matrix(roll, pitch, yaw)
    T_lidar = np.eye(4)
    T_lidar[:3, :3] = R_lidar
    T_lidar[:3, 3] = [t_x, t_y, t_z]

    # 2. Pitch Motor to Yaw Motor (rotation then translation)
    R_pitch = np.array([[cos(beta), 0, sin(beta)],
                        [0, 1, 0],
                        [-sin(beta), 0, cos(beta)]])
    T_pitch_rot = np.eye(4)
    T_pitch_rot[:3, :3] = R_pitch
    
    T_pitch_trans = np.eye(4)
    T_pitch_trans[:3, 3] = [p_x, p_y, p_z_pitch]

    # 3. Yaw Motor to World (rotation then translation)
    R_yaw = np.array([[cos(alpha), -sin(alpha), 0],
                      [sin(alpha),  cos(alpha), 0],
                      [0, 0, 1]])
    T_yaw_rot = np.eye(4)
    T_yaw_rot[:3, :3] = R_yaw
    
    T_world_trans = np.eye(4)
    T_world_trans[:3, 3] = [0, 0, p_z_world]

    # Combined transformation:
    # World = T_world_trans @ T_yaw_rot @ T_pitch_trans @ T_pitch_rot @ T_lidar @ P_lidar
    P_hom = np.append(point, 1)  # Convert point to homogeneous coordinates
    world_point = T_world_trans @ T_yaw_rot @ T_pitch_trans @ T_pitch_rot @ T_lidar @ P_hom
    return world_point[:3]

class LidarCalibrator:
    def __init__(self):
        rospy.init_node('lidar_calibration')
        self.data_points = []
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        # Initial guess for parameters: [p_z_world, p_x, p_y, p_z_pitch, t_x, t_y, t_z, roll, pitch, yaw]
        # Initial guess for parameters: [p_z_world, p_x, p_y, p_z_pitch, t_x, t_y, t_z, roll, pitch, yaw]
        self.initial_params = [1.0, 0.1, 0.1, 0.1, 25, 0.5, 0.5, 0.0, 0.0, 0.0]
        rospy.loginfo("self.initial_params :\n%s", " ".join("{:.6f}".format(x) for x in self.initial_params))


        rospy.Subscriber("/imu_boom_link/data", Imu, self.imu_callback)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_cb)

    def imu_callback(self, msg):
        quat = (msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(quat)  # Returns (roll, pitch, yaw)
        self.current_pitch = euler[1]  # IMU pitch (β)
        self.current_yaw = euler[2]    # IMU yaw (α)

    def lidar_cb(self, msg):
        points = self.extract_points_from_cloud(msg)
        # Check if the returned NumPy array is empty
        if points.size == 0:
            return

        # Randomly sample 500 points from the cloud for efficiency
        num_samples = min(500, len(points))
        sampled_indices = np.random.choice(len(points), num_samples, replace=False)
        for idx in sampled_indices:
            self.data_points.append((self.current_yaw, self.current_pitch, points[idx]))

        rospy.sleep(2)  # Adjust delay as needed


        # Begin calibration when enough points are collected
        if len(self.data_points) > 3000:
            self.run_calibration()

    def run_calibration(self):
        rospy.loginfo("Starting calibration with %d points", len(self.data_points))
        result = least_squares(
            fun=self.residual_function,
            x0=self.initial_params,
            args=(self.data_points,),
            method='lm',  # Levenberg-Marquardt algorithm (unconstrained)
            verbose=2
        )
        rospy.loginfo("Calibration complete!")
        # rospy.loginfo("Optimized parameters:\n%s", result.x)
        rospy.loginfo("Optimized parameters:\n%s", " ".join("{:.6f}".format(x) for x in result.x))

    def residual_function(self, params, data_points):
        residuals = []
        for yaw, pitch, point in data_points:
            world_z = transform_point(params, yaw, pitch, point)[2]
            residuals.append(world_z)  # Ground plane residual (should be zero)
        return np.array(residuals)

    def extract_points_from_cloud(self, cloud_msg):
        """
        Extracts (x, y, z) points from a Velodyne PointCloud2 message.
        """
        points_list = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(points_list) == 0:
            return np.array([])
        return np.array(points_list)

if __name__ == '__main__':
    calibrator = LidarCalibrator()
    rospy.spin()

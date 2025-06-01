import numpy as np
from scipy.optimize import least_squares
from math import sin, cos

def euler_to_rotation_matrix(roll, pitch, yaw):
    # Convert Euler angles to a rotation matrix (ZYX order)
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                   [sin(yaw), cos(yaw), 0],
                   [0, 0, 1]])
    
    Ry = np.array([[cos(pitch), 0, sin(pitch)],
                   [0, 1, 0],
                   [-sin(pitch), 0, cos(pitch)]])
    
    Rx = np.array([[1, 0, 0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])
    
    return Rz @ Ry @ Rx

def transform_point(params, alpha, beta, point):
    # Unpack parameters
    p_z_world = params[0]            # Yaw motor height
    p_x, p_y, p_z_pitch = params[1:4]  # Pitch motor translation
    t_x, t_y, t_z = params[4:7]      # LiDAR translation
    roll, pitch, yaw = params[7:10]   # LiDAR rotation
    
    # LiDAR to Pitch Motor
    R_lidar = euler_to_rotation_matrix(roll, pitch, yaw)
    T_lidar = np.eye(4)
    T_lidar[:3, :3] = R_lidar
    T_lidar[:3, 3] = [t_x, t_y, t_z]
    
    # Pitch rotation
    R_pitch = np.array([[cos(beta), 0, sin(beta)],
                        [0, 1, 0],
                        [-sin(beta), 0, cos(beta)]])
    T_pitch = np.eye(4)
    T_pitch[:3, :3] = R_pitch
    
    # Yaw rotation
    R_yaw = np.array([[cos(alpha), -sin(alpha), 0],
                      [sin(alpha), cos(alpha), 0],
                      [0, 0, 1]])
    T_yaw = np.eye(4)
    T_yaw[:3, :3] = R_yaw
    
    # Full transformation chain
    world_point = (
        np.array([[1, 0, 0, 0],       # Yaw to World translation
                  [0, 1, 0, 0],
                  [0, 0, 1, p_z_world],
                  [0, 0, 0, 1]]) 
        @ T_yaw
        @ np.array([[1, 0, 0, p_x],   # Pitch to Yaw translation
                    [0, 1, 0, p_y],
                    [0, 0, 1, p_z_pitch],
                    [0, 0, 0, 1]])
        @ T_pitch
        @ T_lidar
        @ np.append(point, 1)  # Homogeneous coordinates
    )
    return world_point[:3]

def residual_function(params, data_points):
    residuals = []
    for alpha, beta, point in data_points:
        transformed = transform_point(params, alpha, beta, point)
        residuals.append(transformed[2])  # z-coordinate should be 0
    return np.array(residuals)

# Example usage
if __name__ == "__main__":
    # Initial parameter guesses [p_z_world, p_x, p_y, p_z_pitch, t_x, t_y, t_z, roll, pitch, yaw]
    initial_params = [1.0, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.0, 0.0, 0.0]
    
    # Load collected data (alpha, beta, point)
    # data_points = [...]
    
    # Run optimization
    result = least_squares(
        fun=residual_function,
        x0=initial_params,
        args=(data_points,),
        method='lm',
        verbose=2
    )
    
    print("Optimized parameters:", result.x)

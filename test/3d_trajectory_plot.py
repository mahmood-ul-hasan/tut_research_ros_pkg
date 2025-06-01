#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from matplotlib.offsetbox import AnchoredText

# List of file paths
path = '/home/aisl2/test/'
file_common_name = 'random_3'


file_paths = [
    # path + file_common_name + '/msckf_vio/trj_01/msckf_vio.txt',
    # path + file_common_name + '/vins_fusion_stereo_no_imu/trj_01/vins_fusion_stereo_no_imu.txt',

    path + 'truth/' + file_common_name + '/trj_01.txt',

    # '/home/aisl2/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods/bag_file_2023-09-20-19-09-09/camera_structural_info/trj_01/camera_structural_info.txt'



    # for comparasion differernt VINS methods
    # path + file_common_name + '/camera_structural_info/trj_01/vins_camera_structural_info.txt',
    # path + file_common_name + '/vins_fusion_mono+EKF/trj_01/vins_fusion_mono+EKF.txt',
    # path + file_common_name + '/vins_fusion_stereo/trj_01/vins_fusion_stereo.txt',
    # path + file_common_name + '/vins_fusion_stereo+EKF/trj_01/vins_fusion_stereo+EKF.txt',
    # path + file_common_name + '/vins_mono/trj_01/vins_mono.txt',
    # path + file_common_name + '/vins_mono+EKF/trj_01/vins_mono+EKF.txt',


    # for comparasion proposed methods

    path + file_common_name + '/test2/trj_01/test2.txt',
    # path + file_common_name + '/z_neural_network/trj_01/z_neural_network.txt',

]

# labels = ['ground_truth', 'camera_structural_info', 'vins_fusion_mono+EKF', 'vins_fusion_stereo', 'vins_fusion_stereo+EKF', 'vins_mono', 'vins_mono+EKF']
labels = ['ground_truth', 'camera_structural_info', 'real_sense', 'vins_ekf', 'vins_mono', 'neural_network', 'vins_mono+EKF']
colors = ["orange", "blue", "red", "black", "green", "cyan", "magenta"]

print(file_paths)


def plot_rotation():
    # Create a new figure
    # fig = plt.figure()
    # Create a new figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(6, 6))

    # Iterate over each file
    for file_path, label, color in zip(file_paths, labels, colors):
        # Load data from the file
        data = np.loadtxt(file_path, skiprows=1)

        # Extract columns from the data
        timestamp = data[:, 0]
        qx = data[:, 4]
        qy = data[:, 5]
        qz = data[:, 6]
        qw = data[:, 7]

        # Convert quaternions to rotation matrix
        rotations = Rotation.from_quat(np.vstack((qx, qy, qz, qw)).T)
        euler_angles = rotations.as_euler('xyz', degrees=True)

        # Extract roll, pitch, and yaw angles
        roll = euler_angles[:, 0]
        pitch = euler_angles[:, 1]
        yaw = euler_angles[:, 2]

        # Plot roll, pitch, and yaw in subplots
        axes[0].plot(timestamp, roll, label=label, color=color)
        axes[1].plot(timestamp, pitch, label=label + ' (Pitch)', color=color)
        axes[2].plot(timestamp, yaw, label=label + ' (Yaw)', color=color)

    # Set labels and title for subplots
    axes[0].set_ylabel('Roll (degrees)')
    axes[0].grid(True)
    axes[1].set_ylabel('Pitch (degrees)')
    axes[1].grid(True)
    axes[2].set_ylabel('Yaw (degrees)')
    axes[2].grid(True)
    axes[2].set_xlabel('Timestamp (s)')

    # Set title for the figure
    fig.suptitle('Rotation Angles over Time')

    # Add legends to subplots
    # axes[0].legend()
    # axes[1].legend()
    # axes[2].legend()

    axes[0].legend(
        loc='upper center',
        bbox_to_anchor=(.5, 1.35),
        ncol=2,
    )

    # Save the figure with the desired file name
    fig.savefig(file_common_name + '_rotation_angles_time.png')


def plot_translation():
    # Create a new figure with subplots
    fig3, axes3 = plt.subplots(3, 1, figsize=(6, 6))

    # Iterate over each file
    for file_path, label, color in zip(file_paths, labels, colors):

        # Load data from the file
        data = np.loadtxt(file_path, skiprows=1)

        # Extract columns from the data
        timestamp = data[:, 0]
        tx = data[:, 1]
        ty = data[:, 2]
        tz = data[:, 3]

        # Plot tx, ty, and tz in subplots
        axes3[0].plot(timestamp, tx, label=label, color=color)
        axes3[1].plot(timestamp, ty, label=label + ' (ty)', color=color)
        axes3[2].plot(timestamp, tz, label=label + ' (tz)', color=color)

    # Set labels and title for subplots
    axes3[0].set_ylabel('tx (m)')
    axes3[1].set_ylabel('ty (m)')
    axes3[2].set_ylabel('tz (m)')
    axes3[2].set_xlabel('Timestamp (s)')

    # Set title for the figure
    fig3.suptitle('Translation over Time')

    # Add legends to subplots
    # axes3[0].legend()
    # axes3[1].legend()
    # axes3[2].legend()
    axes3[0].legend(
        loc='upper center',
        bbox_to_anchor=(.5, 1.35),
        ncol=2,)
    axes3[0].grid(True)
    axes3[1].grid(True)
    axes3[2].grid(True)

    # Save the figure with the desired file name
    fig3.savefig(file_common_name + '_translation_time.png')


plot_rotation()
plot_translation()
# Create a 3D figure
fig1 = plt.figure()

ax = fig1.add_subplot(111, projection='3d')

# plot_rotation()
for file_path, label, color in zip(file_paths, labels, colors):

    # Load data from the file
    data = np.loadtxt(file_path, skiprows=1)

    # Extract columns from the data
    timestamp = data[:, 0]
    tx = data[:, 1]
    ty = data[:, 2]
    tz = data[:, 3]

    # Plot the 3D trajectory
    ax.plot(tx, ty, tz, label=label, color=color)
    # ax.plot(tx, ty, tz, label=label)

# Set labels and title
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
# ax.set_title('3D Trajectory')


ax.legend(
    loc='upper center',
    bbox_to_anchor=(.5, 1.15),
    ncol=2,
)
# Save the figure with the desired file name
fig1.savefig(file_common_name + '_3d_traj.png')


# Show the plot
plt.show()

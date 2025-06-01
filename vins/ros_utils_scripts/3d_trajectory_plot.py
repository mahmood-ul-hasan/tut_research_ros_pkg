#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# List of file paths
path = '/home/aisl2/catkin_ws/src/data/model_crane_data/real_sense/'
file_common_name = '23_6_13_boom_only_horizontal_motion'
# file_common_name = '23_6_13_boom_random_motion'

file_paths = [
    path + file_common_name + '_ground_truth.txt',
    path + file_common_name + '_vins_mono.txt',
    path + file_common_name + '_proposed_method.txt'
]

labels = ['ground_truth', 'vins_mono', 'proposed_method']
# labels = ['ground_truth', 'proposed_method' , 'vins_mono']
colors = ['red', 'blue', 'green']

print(file_paths)


# Create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Iterate over each file
# for file_path in file_paths:

for file_path, label, color in zip(file_paths, labels, colors):


      # Extract the desired part of the file path
    # filename = file_path.split('_')[-1].split('.')[0]
    # filename = file_path.split('_')[-2].split('.')[0]


    # Load data from the file
    data = np.loadtxt(file_path, skiprows=1)
    
    # Extract columns from the data
    timestamp = data[:, 0]
    tx = data[:, 1]
    ty = data[:, 2]
    tz = data[:, 3]

    # Plot the 3D trajectory
    ax.plot(tx, ty, tz, label=label, color=color)

# Set labels and title
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_title('3D Trajectory')

# Add a legend
ax.legend()

# Save the figure with the desired file name
fig.savefig(file_common_name + '_fig.png')

# Show the plot
plt.show()

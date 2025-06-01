#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from matplotlib.offsetbox import AnchoredText


file_path = "/home/aisl2/test/truth/random_3/trj_01.txt"

data = np.loadtxt(file_path, skiprows=1)

# Extract columns from the data
timestamp = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]
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

modified_roll = roll + 0
modified_quaternions = np.empty((len(timestamp), 4))

modified_x = x + 10
modified_y = y + 0
modified_z = z + 0
print("X: ", x, "modified_x: ", modified_x)

# Iterate over each timestamp and modify the roll while keeping the original pitch and yaw
for i in range(len(timestamp)):
    modified_rotations = Rotation.from_euler('xyz', [modified_roll[i], pitch[i], yaw[i]], degrees=True)
    modified_quaternions[i] = modified_rotations.as_quat()
    # print(modified_x[i] - x[i])


data[:, 1] = modified_x
data[:, 2] = modified_y
data[:, 3] = modified_z
data[:, 4:8] = modified_quaternions


print("X: ", x, "modified_x: ", modified_x)
fmt = '%.6f'

# Save the modified data to a new text file "test2.txt"
output_file_path = "/home/aisl2/test/random_3/test2/trj_01/test2.txt"
np.savetxt(output_file_path, data, header="# timestamp(s) tx ty tz qx qy qz qw", comments='', delimiter=' ', fmt=fmt)

print("X: ", x, "modified_x: ", modified_x)




# Create subplots
fig, axes = plt.subplots(3, 1, figsize=(6, 6))


# Plot modified position data in red
axes[0].plot(timestamp, modified_x, color='red', label='Modified x')
axes[1].plot(timestamp, modified_y, color='red', label='Modified y')
axes[2].plot(timestamp, modified_z, color='red', label='Modified z')


data = np.loadtxt(file_path, skiprows=1)

# Extract columns from the data
timestamp = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]


# Plot original position data in blue
axes[0].plot(timestamp, x, color='blue', label='Original x')
axes[1].plot(timestamp, y, color='blue', label='Original y')
axes[2].plot(timestamp, z, color='blue', label='Original z')




# Add legends to distinguish the curves
axes[0].legend()
axes[1].legend()
axes[2].legend()

# Set labels and title for subplots
axes[0].set_ylabel('X Position')
axes[1].set_ylabel('Y Position')
axes[2].set_ylabel('Z Position')
axes[2].set_xlabel('Timestamp (s)')



# fig, axes = plt.subplots(3, 1, figsize=(6, 6))
# axes[0].plot(timestamp, roll, color='blue')
# axes[1].plot(timestamp, pitch, color='blue')
# axes[2].plot(timestamp, yaw, color='blue')

# axes[0].plot(timestamp, modified_roll, color='red')
# axes[1].plot(timestamp, pitch)
# axes[2].plot(timestamp, yaw)


# Set labels and title for subplots
# axes[0].set_ylabel('Roll (degrees)')
# axes[0].grid(True)
# axes[1].set_ylabel('Pitch (degrees)')
# axes[1].grid(True)
# axes[2].set_ylabel('Yaw (degrees)')
# axes[2].grid(True)
# axes[2].set_xlabel('Timestamp (s)')

# # Set title for the figure
# fig.suptitle('Rotation Angles over Time')

# Show the plot
plt.show()

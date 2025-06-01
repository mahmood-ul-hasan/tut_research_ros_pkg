import numpy as np

# Define the transformation matrix
matrix = np.array([[0.085562, 0.00867908, 0.996295],
                   [-0.996153, 0.0197568, 0.0853777],
                   [-0.0189426, -0.999767, 0.0103361]])

# Extract the rotation matrix
rotation_matrix = matrix[:3, :3]

# Compute the roll, pitch, and yaw angles
roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

# Convert angles to degrees
roll_deg = np.rad2deg(roll)
pitch_deg = np.rad2deg(pitch)
yaw_deg = np.rad2deg(yaw)

# Print the angles in degrees
print("Roll: {:.2f} degrees".format(roll_deg))
print("Pitch: {:.2f} degrees".format(pitch_deg))
print("Yaw: {:.2f} degrees".format(yaw_deg))
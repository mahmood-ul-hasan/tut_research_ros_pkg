# import numpy as np
# import rospy
# import tf.transformations as tf

# # Quaternion representation of orientation A
# quat_vins_mono = np.array([0.7275777332283623, 0.10323140893859473, -0.11537905989712106, 0.668327457804995])

# print("quat_vins_mono: ", quat_vins_mono)
# # Convert quaternion A to RPY angles
# r_vins_mono = tf.euler_from_quaternion(quat_vins_mono)

# # Print the RPY angles for orientation A
# print("Orientation A (RPY angles):")
# print(f"Roll:  {np.degrees(r_vins_mono[0])} degrees")
# print(f"Pitch: {np.degrees(r_vins_mono[1])} degrees")
# print(f"Yaw:   {np.degrees(r_vins_mono[2])} degrees")
# print("  ")

# # Quaternion representation of orientation B
# quat_gt = np.array([-0.08472752811584812, 0.11503628754738429, -0.6840278057718825, 0.7153277986053649])
# print("quat_gt: ", quat_gt)

# # Convert quaternion B to RPY angles
# r_gt = tf.euler_from_quaternion(quat_gt)
# # Print the RPY angles for orientation B
# print("Orientation B (RPY angles):")
# print(f"Roll:  {np.degrees(r_gt[0])} degrees")
# print(f"Pitch: {np.degrees(r_gt[1])} degrees")
# print(f"Yaw:   {np.degrees(r_gt[2])} degrees")
# print("  ")


# # Calculate the quaternion to add
# quat_add = tf.quaternion_multiply(quat_vins_mono, tf.quaternion_conjugate(quat_gt))

# # Perform quaternion multiplication to obtain orientation B
# quat_after_addition = tf.quaternion_multiply(quat_add, quat_gt)
# print("quat_after_addition: ", quat_after_addition)

# # Convert quaternion B to RPY angles
# r_after_addition = tf.euler_from_quaternion(quat_after_addition)

# # Print the RPY angles for orientation B
# print("Orientation B (RPY angles):")
# print(f"Roll:  {np.degrees(r_after_addition[0])} degrees")
# print(f"Pitch: {np.degrees(r_after_addition[1])} degrees")
# print(f"Yaw:   {np.degrees(r_after_addition[2])} degrees")
# print("  ")


import numpy as np
import rospy
import tf.transformations as tf

# Quaternion representation of orientation A
quat_vins_mono = np.array([0.7275777332283623, 0.10323140893859473, -0.11537905989712106, 0.668327457804995])

print("quat_vins_mono: ", quat_vins_mono)
# Convert quaternion A to RPY angles
r_vins_mono = tf.euler_from_quaternion(quat_vins_mono)

# Print the RPY angles for orientation A
print("Orientation A (RPY angles):")
print(f"Roll:  {np.degrees(r_vins_mono[0])} degrees")
print(f"Pitch: {np.degrees(r_vins_mono[1])} degrees")
print(f"Yaw:   {np.degrees(r_vins_mono[2])} degrees")
print("  ")

# Quaternion representation of orientation B
quat_gt = np.array([-0.08472752811584812, 0.11503628754738429, -0.6840278057718825, 0.7153277986053649])
print("quat_gt: ", quat_gt)

# Convert quaternion B to RPY angles
r_gt = tf.euler_from_quaternion(quat_gt)
# Print the RPY angles for orientation B
print("Orientation B (RPY angles):")
print(f"Roll:  {np.degrees(r_gt[0])} degrees")
print(f"Pitch: {np.degrees(r_gt[1])} degrees")
print(f"Yaw:   {np.degrees(r_gt[2])} degrees")
print("  ")


# Calculate the quaternion to add
quat_add = tf.quaternion_multiply(quat_gt, tf.quaternion_conjugate(quat_vins_mono))
print("quat_add ", quat_add)
# Perform quaternion multiplication to obtain orientation B
quat_after_addition = tf.quaternion_multiply(quat_add, quat_vins_mono)
print("quat_after_addition: ", quat_after_addition)

# Convert quaternion B to RPY angles
r_after_addition = tf.euler_from_quaternion(quat_after_addition)

# Print the RPY angles for orientation B
print("Orientation B (RPY angles):")
print(f"Roll:  {np.degrees(r_after_addition[0])} degrees")
print(f"Pitch: {np.degrees(r_after_addition[1])} degrees")
print(f"Yaw:   {np.degrees(r_after_addition[2])} degrees")
print("  ")

#   x: 0.7275777332283623
#   y: 0.10323140893859473
#   z: -0.11537905989712106
#   w: 0.668327457804995

#   x: -0.08472752811584812
#   y: 0.11503628754738429
#   z: -0.6840278057718825
#   w: 0.7153277986053649

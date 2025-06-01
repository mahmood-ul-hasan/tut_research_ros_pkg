import numpy as np

# List of quaternions
quat_list = [
    [-0.79212187, 0.16593903, -0.31189471, 0.49772369],
    [-0.79432646, 0.15512675, -0.31160461, 0.49787923],
    [-0.79645512, 0.14386749, -0.31137315, 0.49800417],
    [-0.79781168, 0.13605956, -0.31104384, 0.49823293],
    [-0.79968495, 0.12476209, -0.3108928, 0.49828111],
    [-0.80145024, 0.11335392, -0.31056807, 0.49837323]
]

# Convert the list of quaternions to a numpy array
quat_array = np.array(quat_list)

# Compute the average quaternion
quat_avg = np.mean(quat_array, axis=0)

# Print the average quaternion
print("Average Quaternion:", quat_avg)

def save_poses_to_file(n, filename="/media/aisl2/aisl_data/catkin_ws/src/pointcloud_registration/kobelco_exp_2024/pose.json"):
    """
    Generate n rows of transformations in plain text format and save them to a file.

    Parameters:
        n (int): Number of rows to generate.
        filename (str): Name of the file to save the data.
    """
    # Generate the rows
    pose_rows = "0 0 0 1 0 0 0\n" * n

    # Save to the file
    with open(filename, "w") as file:
        file.write(pose_rows.strip())  # Strip to remove the last extra newline

    print(f"Saved {n} rows to {filename} in plain text format.")

# Example usage
n = 287  # Number of rows
save_poses_to_file(n)

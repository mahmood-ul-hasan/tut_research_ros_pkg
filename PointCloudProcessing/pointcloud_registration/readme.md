# Point Cloud Registration and Merging Framework

This project provides a robust framework for point cloud registration, alignment, and merging using the **Point Cloud Library (PCL)**. It is designed to efficiently process point clouds, making it suitable for tasks such as computing scanner odometry and merging multiple point clouds into a single global reference frame. This framework can be applied to the **Vitom Technical Assignment** described below.

---

## Task Description: Vitom Technical Assignment

The assignment involves processing 70 point cloud files (.pcd) recorded with a handheld scanner. The files have been pre-deskewed using IMU data. Your goal is to:

1. **Sequentially register the point clouds** to compute the odometry of the scanner.
2. **Merge the point clouds** into a single global reference frame based on the first scan.
3. Output the merged point cloud as a single `.pcd` file, capturing as much of the data as possible.

Link to the point cloud files: [Scans.zip](https://www.dropbox.com/scl/fi/5xkskqoxw6p1rcyitg6l8/scans.zip?rlkey=1u68p3w8qsxwa7rbq2mpxhubm&dl=0)

You can use any library you are comfortable with, such as **PCL** or **Open3D**.

---
## Prerequisites Libraries

- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [Boost Libraries](https://www.boost.org/)
- [Eigen](https://eigen.tuxfamily.org/)
- [Matplotlib-cpp](https://github.com/lava/matplotlib-cpp)
- [Robot Operating System (ROS)](https://www.ros.org/) for odometry and cloud publishing.
ROS1 Noetic


## Without ROS:
---

1. Clone the Repository


```bash
Copy the folder to your PC:
mkdir build
cd build
cmake ..
make

./sequential_registration_with_out_ros
```

## With ROS:
---

```bash
catkin_make
source devel/setup.bash
roslaunch pointcloud_registration sequential_registration.launchmake
```

## Configuration Options:

### Global Registration Options:
Set the `globalAlignmentMethod` variable to one of the following:

- `"icp"`: Iterative Closest Point.
- `"gicp"`: Generalized Iterative Closest Point.
- `"ndt"`: Normal Distributions Transform.
- `"none"`: Direct merging without alignment.

### Merging Options:
Modify the `mergeAllAtOnce` flag:

- `true`: Merge all clouds in a single pass.
- `false`: Merge clouds incrementally in subsets.



## Outputs:

1. **Merged Point Cloud**:  
   The final merged cloud is saved as `merged_cloud.pcd` in the `given_pcd_data` folder.

2. **Odometry Data**:  
   Translations (X, Y, Z) and rotations (Roll, Pitch, Yaw) are saved in `odom_data.txt`  in the `given_pcd_data` folder.

3. **Visualization**:  
   2D plots for translation and rotation metrics are generated using `matplotlib-cpp`.


## Example Workflow:

1. **Load Point Clouds**:  
   Use `loadCloudData` to load `.pcd` files, remove NaN points, and prepare the clouds.

2. **Filter and Align**:  
   Align each pair of point clouds using the selected registration method.

3. **Merge**:  
   Incrementally merge aligned clouds into a global point cloud.

4. **Analyze**:  
   Visualize fitness scores, translations, and rotations for evaluation.

5. **Save Results**:  
   Save the merged point cloud and odometry data.


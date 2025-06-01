import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def read_pcd(file_path):
    # Load point cloud from .pcd file
    pcd = o3d.io.read_point_cloud(file_path)

    print("Point Cloud Fields:")
    for attr in dir(pcd):
        if not attr.startswith('__'):
            print(f"- {attr}")
    return pcd

def assign_colors_based_on_intensity(pcd):
    # Convert point cloud to numpy array
    points = np.asarray(pcd.points)
    print("points: ", points)
    
    # Check if colors are present
    if not pcd.has_colors():
        print("Point cloud does not have color information.")
        return pcd
    
    intensities = np.asarray(pcd.colors)[:, 0]  # Assuming intensity is stored in the red channel
    
    # Check if intensities array is empty
    if len(intensities) == 0:
        print("Intensity array is empty.")
        return pcd
    
    # Normalize intensities to [0, 1] range
    min_intensity = np.min(intensities)
    max_intensity = np.max(intensities)
    normalized_intensities = (intensities - min_intensity) / (max_intensity - min_intensity)

    # Assign colors based on normalized intensities using a colormap
    colormap = plt.get_cmap("jet")
    colors = colormap(normalized_intensities)[:, :3]  # Get RGB values from colormap

    # Update the colors in the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # Path to your .pcd file
    file_path = '/media/aisl2/aisl_data/catkin_ws/src/latest_crane/laser_to_pcl/src/PCD/capture00001.pcd'
    
    pcd = read_pcd(file_path)
    print(pcd)
    pcd = assign_colors_based_on_intensity(pcd)
    visualize_point_cloud(pcd)

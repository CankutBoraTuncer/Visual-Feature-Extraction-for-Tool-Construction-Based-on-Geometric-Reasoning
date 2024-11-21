import open3d as o3d
import numpy as np
from sklearn.cluster import AgglomerativeClustering
import matplotlib.pyplot as plt

# Load the point cloud
point_cloud = o3d.io.read_point_cloud("../src/point_clouds_ref/shovel_1.ply")
if len(point_cloud.points) == 0:
    raise ValueError("Point cloud is empty. Check the file.")

# Preprocessing
print("Preprocessing")
point_cloud = point_cloud.voxel_down_sample(voxel_size=0.0005)
print(f"Downsampled point cloud has {len(point_cloud.points)} points.")
point_cloud, _ = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Segment the handle
print("Segment the handle")
plane_model, handle_inliers = point_cloud.segment_plane(distance_threshold=0.008, ransac_n=3, num_iterations=2000)
handle_segment = point_cloud.select_by_index(handle_inliers)
remaining_points = point_cloud.select_by_index(handle_inliers, invert=True)

segmented_plane = point_cloud.select_by_index(handle_inliers)
remaining_points = point_cloud.select_by_index(handle_inliers, invert=True)
segmented_plane.paint_uniform_color([1.0, 0.0, 0.0])  # Red for plane
remaining_points.paint_uniform_color([0.0, 0.0, 1.0])  # Blue for remaining
print("Segmented Plane and Remaining Points:")
o3d.visualization.draw_geometries([segmented_plane, remaining_points])


points = np.asarray(remaining_points.points)
clustering = AgglomerativeClustering(n_clusters=2).fit(points)
labels = clustering.labels_

# Assign labels to the point cloud
colors = plt.get_cmap("tab20")(labels / (max(labels) + 1))
remaining_points.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([remaining_points])


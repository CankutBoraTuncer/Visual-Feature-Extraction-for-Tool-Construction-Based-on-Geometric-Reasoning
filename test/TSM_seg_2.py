import sys
sys.path.append('../src')
import os

import pyvista as pv
import numpy as np
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler



point_cloud_path = "../src/point_clouds_ref"
model_names = os.listdir(point_cloud_path)

for model_name in model_names:
    print("Model name:", model_name)
    
    # Load the point cloud from a .ply file
    point_cloud = pv.read(point_cloud_path + "/" + model_name)

    # Extract points (vertices) from the cleaned point cloud
    vertices = point_cloud.points

    # Perform K-Means Clustering
    kmeans = KMeans(
        n_clusters=2,       # Number of clusters
        init="k-means++",   # Initialization method
        n_init=20,          # Number of runs with different seeds
        max_iter=600,       # Maximum iterations
        random_state=42     # Ensure reproducibility
    )

    labels = kmeans.fit_predict(vertices)

    color_map = {0: "part 0", 1: "part 1"}

    cluster_colors = [color_map[label] for label in labels]

    # Assign cluster labels to the cleaned point cloud
    point_cloud["Labels"] = cluster_colors

    # Visualize segmented point cloud
    point_cloud.plot(scalars="Labels", cmap="tab10")

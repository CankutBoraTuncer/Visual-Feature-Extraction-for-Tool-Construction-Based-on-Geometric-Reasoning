import sys
sys.path.append('../src')
import os


import pyvista as pv
import numpy as np
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler

# Path to models
model_path = "../src/models"
model_names = os.listdir(model_path)

for model_name in model_names:
    print("Processing Model:", model_name)

    # Load the STL file
    mesh = pv.read(os.path.join(model_path, model_name))

    # Clean the mesh
    clean_mesh = mesh.clean()

    # Extract vertices
    vertices = clean_mesh.points

    # Normalize vertices for clustering
    scaler = StandardScaler()
    vertices_normalized = scaler.fit_transform(vertices)

    # Determine the number of clusters dynamically or set it manually
    n_clusters = 2  # Adjust as needed

    # Perform K-Means Clustering
    kmeans = KMeans(
        n_clusters=n_clusters,
        init="k-means++",
        n_init=10,
        max_iter=300,
        random_state=42
    )

    # Predict cluster labels
    labels = kmeans.fit_predict(vertices)

    # Assign numeric labels to the mesh
    clean_mesh["Labels"] = labels

    # Define color map for visualization
    color_map = {i: f"Cluster {i}" for i in range(n_clusters)}
    cluster_colors = [color_map[label] for label in labels]

    # Visualize the segmented model
    print(f"Cluster Summary: {color_map}")
    clean_mesh.plot(scalars="Labels")
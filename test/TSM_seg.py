import pyvista as pv
import numpy as np
from sklearn.cluster import KMeans

# Load the STL file
mesh = pv.read("../src/models/hammer_large.stl")

# Clean the mesh
clean_mesh = mesh.clean()

# Extract vertices
vertices = clean_mesh.points

# Perform K-Means Clustering
kmeans = KMeans(n_clusters=2, random_state=42)
labels = kmeans.fit_predict(vertices)

# Define colors for each cluster
color_map = {0: "blue", 1: "red"}
cluster_colors = [color_map[label] for label in labels]

# Assign cluster labels to the mesh
clean_mesh["Labels"] = cluster_colors

# Visualize segmented parts
clean_mesh.plot(scalars="Labels")
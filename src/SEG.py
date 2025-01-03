from sklearn.cluster import KMeans
import open3d as o3d
import numpy as np
import robotic as ry
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN
import pyvista as pv
from RAI import RAI

class SEG():
    def __init__(self, verbose=0):
        self.verbose = verbose

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def euclidean_dist(self, a, b):
        return np.sqrt(np.sum((a - b) ** 2))
    
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def region_query(self, X, point_idx, eps, kdtree):
        point = X[point_idx]
        [_, idxs, _] = kdtree.search_radius_vector_3d(point, eps)
        return idxs
    
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def expand_cluster(self, X, labels, point_idx, neighbors, cluster_id, eps, min_samples, kdtree):
        labels[point_idx] = cluster_id
        i = 0
        while i < len(neighbors):
            n_point_idx = neighbors[i]
            if labels[n_point_idx] == -1:
                labels[n_point_idx] = cluster_id
            elif labels[n_point_idx] == 0:
                labels[n_point_idx] = cluster_id
                n_neighbors = self.region_query(X, n_point_idx, eps, kdtree)
                if len(n_neighbors) >= min_samples:
                    neighbors.extend(n_neighbors)
            i += 1

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def DBSCAN(self, X, eps, min_samples):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(X)
        kdtree = o3d.geometry.KDTreeFlann(pcd)

        labels = np.zeros(len(X), dtype=int)
        cluster_id = 0

        for i in range(len(X)):
            if labels[i] != 0:
                continue

            neighbors = self.region_query(X, i, eps, kdtree)
            if len(neighbors) < min_samples:
                labels[i] = -1
            else:
                cluster_id += 1
                self.expand_cluster(X, labels, i, neighbors, cluster_id, eps, min_samples, kdtree)

        return labels
    
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def segment_point_cloud(self, point_cloud, n_clusters=2, init="k-means++", n_init=20, max_iter=600, random_state=42, is_save=False, save_path=None):
        vertices = point_cloud.points

        kmeans = KMeans(
            n_clusters=n_clusters,      
            init=init,                 
            n_init=n_init,              
            max_iter=max_iter,            
            random_state=random_state   
        )

        labels = kmeans.fit_predict(vertices)
        point_cloud["Labels"] = labels

        if(self.verbose > 0):
            point_cloud.plot(scalars="Labels", cmap="tab10")
        
        if(is_save):
            point_cloud.save(save_path)
            print("Point cloud saved to:", save_path)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def segment_objects(self, point_cloud, scene, eps=0.02, min_samples=10, is_save=False, save_path=None):
        vertices = np.array(point_cloud.points)
        labels = self.DBSCAN(vertices, eps, min_samples)
        point_cloud["Labels"] = labels

        if self.verbose > 0:
            print(f"Number of clusters (excluding noise): {len(set(labels)) - (1 if -1 in labels else 0)}")
            print(f"Number of noise points: {np.sum(labels == -1)}")
            point_cloud.plot(scalars="Labels", cmap="tab10")

        if is_save:
            point_cloud.save(save_path)
            print("Point cloud saved to:", save_path)

        segments = {}
        unique_labels = np.unique(labels)
        unique_labels = unique_labels[unique_labels != -1]

        for lbl in unique_labels:
            mask = (labels == lbl)
            segment_points = vertices[mask]

            new_cloud = o3d.geometry.PointCloud()
            new_cloud.points = o3d.utility.Vector3dVector(segment_points)

            segments[RAI.point2obj(scene, segment_points)] = new_cloud

        return segments
                  
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def segment_point_cloud_v2(self, path, radius=0.02):
        def compute_eigenvalues_and_descriptors(point_cloud, radius):
            pcd_tree = o3d.geometry.KDTreeFlann(point_cloud)
            points = np.asarray(point_cloud.points)
            descriptors = []

            for i in range(points.shape[0]):
                [_, idx, _] = pcd_tree.search_radius_vector_3d(points[i], radius)
                if len(idx) < 3: 
                    descriptors.append((0, 0, 0, 0, 0))
                    continue

                neighbors = points[idx]
                cov_matrix = np.cov(neighbors.T)

                eigenvalues, _ = np.linalg.eig(cov_matrix)
                eigenvalues = np.sort(eigenvalues)[::-1] 

                eigen_sum = np.sum(eigenvalues)
                eigenvalues = eigenvalues / eigen_sum if eigen_sum != 0 else eigenvalues

                linearity = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0] if eigenvalues[0] != 0 else 0
                planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0] if eigenvalues[0] != 0 else 0
                sphericity = eigenvalues[2] / eigenvalues[0] if eigenvalues[0] != 0 else 0

                descriptors.append((linearity, planarity, sphericity, eigenvalues[0], eigenvalues[1]))

            return np.array(descriptors)

        colors = {
            0: [1, 0, 0],  # Red for cylindrical
            1: [0, 1, 0],  # Green for planar
            2: [0, 0, 1],  # Blue for prismatic
            3: [1, 1, 0],  # Yellow for spherical
            -1: [0.5, 0.5, 0.5]  # Gray for uncertain
        }
            
        def classify_points(descriptors):
            labels = []
            for linearity, planarity, sphericity, _, _ in descriptors:
                if linearity > 0.7 and planarity < 0.2:  # Cylindrical (Handles: Hammer, Shovel, Knife)
                    print("Cylindrical", linearity, planarity, sphericity)
                    labels.append(0)
                elif planarity > 0.5 and linearity < 0.2:  # Planar (Blades: Scraper, Shovel, Spatula)
                    print("Planar", linearity, planarity, sphericity)
                    labels.append(1)
                elif planarity > 0.2 and sphericity < 0.2:  # Prismatic (Handles/Heads: Knife, Hammer, Spatula)
                    print("Prismatic", linearity, planarity, sphericity)
                    labels.append(2)
                elif sphericity > 0.6:  # Spherical/Joint-like (Plier joint)
                    print("Spherical", linearity, planarity, sphericity)
                    labels.append(3)
                else:  # Other/Uncertain
                    print("Uncertain", linearity, planarity, sphericity)
                    labels.append(-1)
            return np.array(labels)

        pcd = o3d.io.read_point_cloud(path)

        descriptors = compute_eigenvalues_and_descriptors(pcd, radius)

        labels = classify_points(descriptors)

        points = np.asarray(pcd.points)

        segmented_points = {
            0: [],  # Cylindrical
            1: [],  # Planar
            2: [],  # Prismatic
            3: [],  # Spherical
            -1: []  # Uncertain
        }

        for i, label in enumerate(labels):
            segmented_points[label].append(points[i])

        point_clouds = []
        for label, points_list in segmented_points.items():
            if points_list:  
                segment_pcd = o3d.geometry.PointCloud()
                segment_pcd.points = o3d.utility.Vector3dVector(points_list)
                segment_pcd.paint_uniform_color(colors[label])
                point_clouds.append(segment_pcd)

        o3d.visualization.draw_geometries(point_clouds)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#
    
    def segment_mesh(self, mesh, n_clusters=2, init="k-means++", n_init=10, max_iter=300, random_state=42, is_save=False, save_path=None):
        clean_mesh = mesh.clean()
        vertices = clean_mesh.points

        kmeans = KMeans(
            n_clusters=n_clusters,
            init=init,
            n_init=n_init,
            max_iter=max_iter,
            random_state=random_state
        )

        labels = kmeans.fit_predict(vertices)
        clean_mesh["Labels"] = labels

        if(self.verbose > 0):
            clean_mesh.plot(scalars="Labels")
        
        if(is_save):
            clean_mesh.save(save_path)
            print("Mesh saved to:", save_path)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def RANSAC_plane(self, pcl, iteration=20, dist_th=0.01):
        H_best = None
        inliers_best = []
        distance_threshold = dist_th

        for c in range(iteration): 
            subset_count = 3 
            if len(inliers_best) >= 8:
                subset_idx = np.random.choice(inliers_best, int(len(inliers_best) / 2), replace=False)
            else:
                subset_idx = np.random.choice(len(pcl), subset_count, replace=False)
            
            subset = pcl[subset_idx]
            
            p1, p2, p3 = subset[0], subset[1], subset[2]
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            normal = normal / np.linalg.norm(normal)
            
            a, b, c = normal
            d = -np.dot(normal, p1)

            inliers = []
            for i in range(len(pcl)):
                x, y, z = pcl[i]
                distance = abs(a * x + b * y + c * z + d) / np.sqrt(a**2 + b**2 + c**2)
                if distance < distance_threshold:
                    inliers.append(i)

            if len(inliers) > len(inliers_best):
                inliers_best = inliers
                H_best = (a, b, c, d)

        inliers_set = set(inliers_best)
        pcl_idx = [i for i in range(len(pcl)) if i not in inliers_set]

        pcl_filtered = pcl[pcl_idx]
        pcl_filtered = np.asarray(pcl_filtered)

        if self.verbose > 0:
            print("Number of Inliers:", len(inliers_best))
            print("Best Plane Coefficients: a, b, c, d =", H_best)
            C_view = ry.Config()
            C_view.addFrame("world")
            C_view.getFrame("world").setPointCloud(pcl_filtered.flatten(), [0,0,0])
            C_view.view(True)
        
        return pv.PolyData(pcl_filtered)
    


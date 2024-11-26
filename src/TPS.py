from sklearn.cluster import KMeans
import open3d as o3d
import numpy as np

class TPS():

    def __init__(self, verbose=0):
        self.verbose = verbose

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
        color_map = {0: "part 0", 1: "part 1"}
        cluster_colors = [color_map[label] for label in labels]
        point_cloud["Labels"] = cluster_colors

        if(self.verbose > 0):
            point_cloud.plot(scalars="Labels", cmap="tab10")
        
        if(is_save):
            point_cloud.save(save_path)
            print("Point cloud saved to:", save_path)

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
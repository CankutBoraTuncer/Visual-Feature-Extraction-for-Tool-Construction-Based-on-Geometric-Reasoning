import sys
sys.path.append('../src')
import os

from rai import rai
import robotic as ry
import open3d as o3d
import numpy as np

if __name__ == "__main__":
    cam_list = ["cam_front", "cam_back", "cam_left", "cam_right", "cam_up"]
    filter = 1
    model_path = "../src/models/simple/parts"
    model_names = os.listdir(model_path)

    for model_name in model_names:
        model_name = model_name.split(".")[0]
        print("Model name:", model_name)
        base_arg = "X: [0.0, 0.0, 0.2, 0.7, 0, 0.7, 0], color: [0, 1, 1], contact: 1, shape: mesh, visual: True, mesh: <../src/models/simple/parts/"
        arg = base_arg + model_name + ".stl>,"

        C = ry.Config()
        C.addFile("../src/config/plane.g")
        C.addFile("../src/config/base.g")
        C.addFrame(model_name, "world", args=arg)  

        r = rai(C, cam_list, view=False)
        pcl, _, _ = r.get_raw_point_cloud(img_view=False, filter = filter)
        print(pcl.shape)

        H_best = None
        inliers_best = []
        distance_threshold = 0.01  # Adjust as needed

        for c in range(20):  # Number of iterations
            print(c)
            subset_count = 3  # Minimum points to define a plane
            if len(inliers_best) >= 8:
                subset_idx = np.random.choice(inliers_best, int(len(inliers_best) / 2), replace=False)
            else:
                subset_idx = np.random.choice(len(pcl), subset_count, replace=False)
            
            subset = pcl[subset_idx]
            
            # Fit plane using three points
            p1, p2, p3 = subset[0], subset[1], subset[2]
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
            
            # Plane equation coefficients: ax + by + cz + d = 0
            a, b, c = normal
            d = -np.dot(normal, p1)

            # Evaluate inliers
            inliers = []
            for i in range(len(pcl)):
                x, y, z = pcl[i]
                distance = abs(a * x + b * y + c * z + d) / np.sqrt(a**2 + b**2 + c**2)
                if distance < distance_threshold:
                    inliers.append(i)

            # Update the best plane if more inliers are found
            if len(inliers) > len(inliers_best):
                inliers_best = inliers
                H_best = (a, b, c, d)

        print("Number of Inliers:", len(inliers_best))
        print("Best Plane Coefficients: a, b, c, d =", H_best)

        inliers_set = set(inliers_best)

        # Use list comprehension to find indices not in inliers_best
        pcl_idx = [i for i in range(len(pcl)) if i not in inliers_set]

        pcl_filtered = pcl[pcl_idx]
        pcl_filtered = np.asarray(pcl_filtered)
        C_view2 = ry.Config()
        print(len(pcl_filtered))
        C_view2.addFrame("world")
        C_view2.getFrame("world").setPointCloud(pcl_filtered.flatten(), [0,0,0])
        C_view2.view(True)

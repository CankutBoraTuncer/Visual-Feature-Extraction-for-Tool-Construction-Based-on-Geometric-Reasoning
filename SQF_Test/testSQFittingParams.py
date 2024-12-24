import SQF
import numpy as np
from scipy.optimize import least_squares
import open3d as o3d
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize, Bounds
import copy
import matplotlib.pyplot
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file_path = "/Users/hsimsir/Documents/CS554_Computer_Vision_Project/output_point_cloud_4.ply"
ml_file_path = "/Users/hsimsir/Documents/CS554_Computer_Vision_Project/ml_output_point_cloud_4.ply"

pcd = o3d.io.read_point_cloud(file_path)
pcd_ml = o3d.io.read_point_cloud(ml_file_path)

points_ml = np.asarray(pcd_ml.points)

# pcd = normalize_y_axis(pcd)
points = np.asarray(pcd.points)
if len(pcd.points) == 0:
    print(f"zero pcd")

pca = PCA(n_components=3)
pca.fit(points)
rotation_matrix = pca.components_
orientations = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
points = points @ rotation_matrix.T  # Apply rotation
pcd.points = o3d.utility.Vector3dVector(points)
SQF.normalize_point_cloud(pcd)
points = np.asarray(pcd.points)

pca = PCA(n_components=3)
pca.fit(points_ml)
rotation_matrix = pca.components_
orientations = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
points_ml = points_ml @ rotation_matrix.T  # Apply rotation
pcd_ml.points = o3d.utility.Vector3dVector(points_ml)
SQF.normalize_point_cloud(pcd_ml)
points_ml = np.asarray(pcd_ml.points)

o3d.io.write_point_cloud(ml_file_path, pcd_ml)

fit_params, fit_type, SQ, residue_SQ = SQF.SQ_fitting_params(points, [0])

SQF.visualizePointClouds([points, SQ], ["Original PC",f"Our SQ Fitting. Residue: {residue_SQ:.4f}"],
                        title="Fitted Ellipsoid and the Original PC (tong pc)")
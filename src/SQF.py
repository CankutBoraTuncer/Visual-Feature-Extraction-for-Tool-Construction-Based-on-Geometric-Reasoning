import numpy as np
import open3d as o3d
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize, Bounds
import copy
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from scipy.optimize import least_squares

class SQF():
    def __init__(self, pcd, verbose = 0):
        self.pcd = pcd
        self.iteration_count = 0
        self.verbose = verbose

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#
    
    def fit(self, sq_type):
        points = np.asarray(self.pcd.points)
        pca = PCA(n_components=3)
        pca.fit(points)
        rotation_matrix = pca.components_
        points = points @ rotation_matrix.T


        self.pcd.points = o3d.utility.Vector3dVector(points)
        self.normalize_point_cloud(self.pcd)
        points = np.asarray(self.pcd.points)
        
        if self.verbose > 0:
            self.display_pcd(self.pcd, "initial pcd")

        filtered_points = points 
        if self.verbose > 0:
            self.display_pcd(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(filtered_points)), "filtered pcd")

        return self.SQ_fitting_params(filtered_points, sq_type)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#
    
    def visualize(self, obj, window_name='Open3D'):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name, width=800, height=600)
        vis.add_geometry(obj)
        vis.run()
        vis.destroy_window()

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def normalize_point_cloud(self, pcd):
        points = np.asarray(pcd.points)
        center = points.mean(axis=0)
        points -= center
        max_abs_value = np.max(np.abs(points))
        points /= max_abs_value
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    SQTYPES=["superellipsoid", "hyperboloid", "toroid", "paraboloid"]

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def visualizePointClouds(self, PCList, labels, title="Point Clouds"):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        cmap = get_cmap("tab20")  
        num_colors = len(PCList)

        for i, pcl in enumerate(PCList):
            color = cmap(i / num_colors) 
            ax.scatter(
                pcl[:, 0],
                pcl[:, 1],
                pcl[:, 2],
                s=3,
                label=labels[i],
                alpha=0.6,
                c=[color]  
            )

        ax.set_title(title)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        ax.legend()
        self.set_axes_equal(ax)
        plt.show()

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def set_axes_equal(self, ax):
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d()
        ])
        ranges = np.abs(limits[:, 1] - limits[:, 0])
        max_range = ranges.max()

        midpoints = np.mean(limits, axis=1)
        ax.set_xlim3d([midpoints[0] - max_range / 2, midpoints[0] + max_range / 2])
        ax.set_ylim3d([midpoints[1] - max_range / 2, midpoints[1] + max_range / 2])
        ax.set_zlim3d([midpoints[2] - max_range / 2, midpoints[2] + max_range / 2])

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def SQ_fitting_params(self, point_cloud, SQ_type):

        segment_members = copy.deepcopy(point_cloud)
        segment_members_original = copy.deepcopy(segment_members)
        residue_SQ = float('inf')
        
        for sq_type in SQ_type:
            scale, orientations, eps, p, bound_min, bound_max = self.param_init(segment_members, sq_type)

            lower_bounds = [
                0.8 * scale[0], 0.8 * scale[1], 0.8 * scale[2], 0.8 * scale[3] - 1e-2, 0.1, 0.1,
                -np.pi, -np.pi, -np.pi, bound_min[0], bound_min[1], bound_min[2]
            ]
            upper_bounds = [
                1.2 * scale[0], 1.2 * scale[1], 1.2 * scale[2], 1.2 * scale[3] + 1e-2, 2.0, 2.0,
                np.pi, np.pi, np.pi, bound_max[0], bound_max[1], bound_max[2]
            ]
            bounds = Bounds(lower_bounds, upper_bounds)

            x_init = np.array([
                scale[0], scale[1], scale[2], scale[3], eps[0], eps[1],
                orientations[0], orientations[1], orientations[2],
                p[0], p[1], p[2]
            ])


            x_init = [float(x) for x in x_init]
            x_init[4:] = [x + 1e-6 if abs(x) < 1e-6 else x for x in x_init[4:]]

            if np.any(lower_bounds >= upper_bounds):
                if self.verbose > 0:
                    print(f"lowers are not strictly lower")
                assert False

            for i, (x, lb, ub) in enumerate(zip(x_init, lower_bounds, upper_bounds)):
                if not (lb <= x <= ub):
                    if self.verbose > 0:
                        print(f"Parameter {i} out of bounds: {x} not in [{lb}, {ub}]")
                    assert False
            if self.verbose > 0:
                print(f"x_init: {x_init}")
                print(f"lower_bound: {lower_bounds}")
                print(f"upper_bound: {upper_bounds}")

            def residuals_fn(params):
                return self.fitting_fn(params, segment_members, sq_type)
            
            result = least_squares(
                fun=residuals_fn,
                x0=x_init,
                bounds=bounds,
                method='trf',  
                max_nfev=5000,
                ftol=1e-8,
                xtol=1e-8,
                verbose=1
            )

            if not result.success:
                raise ValueError(f"Optimization failed: {result.message}")

            optimum_quadrics = result.x

            SQ = self.SQ2PCL(optimum_quadrics, sq_type)

            pcl_SQ_dist, SQ_pcl_dist = self.pcl_dist(segment_members_original, SQ)
            residue = pcl_SQ_dist + SQ_pcl_dist

            if self.verbose > 0:
                print(f"residue is for type {sq_type} is {residue}")
                print(f"params for type: {sq_type} is: {optimum_quadrics}")

            if residue < residue_SQ:
                SQ_optimum = optimum_quadrics
                residue_SQ = residue
                optimum_type = sq_type

        if self.verbose > 0:
            print(f"SQ_optimum: {SQ_optimum}")

        SQ = self.SQ2PCL(SQ_optimum, optimum_type)
        if self.verbose > 1:
            self.visualizePointClouds([SQ, segment_members_original], [f"BEST SQ of type: {sq_type}", "original ptc"])

        if optimum_type == 2:  
            fit_params = copy.deepcopy(SQ_optimum)
        else:
            fit_params = np.zeros(11)
            fit_params[:3] = SQ_optimum[:3]
            fit_params[3:5] = SQ_optimum[4:6]
            fit_params[5:8] = SQ_optimum[6:9]
            fit_params[8:11] = SQ_optimum[9:12]

        fit_type = optimum_type

        return fit_params, fit_type, SQ, residue_SQ

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def pca_segment(self, pcl):
        segments = np.array(pcl)

        pca = PCA(n_components=3)  
        pca.fit(segments)    

        pca_segments = pca.components_.T  

        new_segments = segments @ pca_segments

        inv_pca = np.linalg.inv(pca_segments)

        return new_segments, inv_pca

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def param_init(self, segment, sq_type):

        scale = np.zeros(4)
        segment_members = segment

        pca = PCA(n_components=3)
        pca.fit(segment_members)
        rotation_matrix = pca.components_
        orientations = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)

        minimum = np.min(segment_members, axis=0)
        maximum = np.max(segment_members, axis=0)

        bound_min = minimum
        bound_max = maximum


        pcl_scale = np.abs(maximum - minimum)
        pcl_scale = np.where(pcl_scale == 0, pcl_scale + 0.000005, pcl_scale)

        eps = [0.1, 1.0]  

        if sq_type != 3 and pcl_scale[2] > 0.25 * pcl_scale[1]:
            z_scale = pcl_scale[0]
            pcl_scale[0] = pcl_scale[2]
            pcl_scale[2] = z_scale
            orientations[1] += np.pi / 2  

        scale[:3] = pcl_scale / 2  

        if sq_type == 2:  
            scale[3] = 0.02  

        p = np.mean(segment_members, axis=0)

        return scale, orientations, eps, p, bound_min, bound_max

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def fitting_fn(self, opt_params, current_segment, SQ_type):
        pcl = current_segment

        a1, a2, a3, a4 = opt_params[:4]


        eps1, eps2 = opt_params[4:6]
        eps1 = max(eps1, 1e-6)  
        eps2 = max(eps2, 1e-6)  

        angle_x, angle_y, angle_z = opt_params[6:9]

        trans_x, trans_y, trans_z = opt_params[9:12]

        nx = np.cos(angle_x) * np.cos(angle_y) * np.cos(angle_z) - (np.sin(angle_x) * np.sin(angle_z))
        ny = np.sin(angle_x) * np.cos(angle_y) * np.cos(angle_z) + np.cos(angle_x) * np.sin(angle_z)
        nz = -(np.sin(angle_y) * np.cos(angle_z))
        ox = -(np.cos(angle_x) * np.cos(angle_y) * np.sin(angle_z)) - (np.sin(angle_x) * np.cos(angle_z))
        oy = -(np.sin(angle_x) * np.cos(angle_y) * np.sin(angle_z)) + np.cos(angle_x) * np.cos(angle_z)
        oz = np.sin(angle_y) * np.sin(angle_z)
        ax = np.cos(angle_x) * np.sin(angle_y)
        ay = np.sin(angle_x) * np.sin(angle_y)
        az = np.cos(angle_y)

        X = (nx * pcl[:, 0] + ny * pcl[:, 1] + nz * pcl[:, 2] - trans_x * nx - trans_y * ny - trans_z * nz)
        Y = (ox * pcl[:, 0] + oy * pcl[:, 1] + oz * pcl[:, 2] - trans_x * ox - trans_y * oy - trans_z * oz)
        Z = (ax * pcl[:, 0] + ay * pcl[:, 1] + az * pcl[:, 2] - trans_x * ax - trans_y * ay - trans_z * az)

        a1 = max(a1, 1e-6)
        a2 = max(a2, 1e-6)
        a3 = max(a3, 1e-6)

        X = np.abs(X) / a1
        Y = np.abs(Y) / a2
        Z = np.abs(Z) / a3

        X1 = X ** (2 / eps2)
        Y1 = Y ** (2 / eps2)
        Z1 = Z ** (2 / eps1)
        
        if SQ_type in [0, 1]:
            quadric_func = (X1 + Y1) ** (eps2 / eps1) + Z1
        elif SQ_type == 2:
            quadric_func = np.abs(((X1 + Y1) ** (eps2 / 2) - a4)) ** (2 / eps1) + Z1
        elif SQ_type == 3:
            quadric_func = np.abs(((X1 + Y1) ** (eps2 / eps1)) - Z)
        else:
            raise ValueError("Incorrect SQ_type input")

        cost = (a1 * a2 * a3) * (np.abs(quadric_func ** eps1) - 1) ** 2

        if np.any(np.isnan(cost)) or np.any(np.isinf(cost)):
            cost = np.full_like(cost, 1e12)

        return cost

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def SQ2PCL(self, params, SQ_type):
        n = 50  

        a1 = params[0]
        a2 = params[1]
        a3 = params[2]

        if len(params) > 11:
            a4 = params[3]
            eps1 = params[4]
            eps2 = params[5]
        else:
            eps1 = params[3]
            eps2 = params[4]

        angle_x = params[-6]
        angle_y = params[-5]
        angle_z = params[-4]
        trans_x = params[-3]
        trans_y = params[-2]
        trans_z = params[-1] 

        if SQ_type == 0: 
            eta = np.linspace(-np.pi / 2, np.pi / 2, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * np.sign(np.sin(eta)) * (np.abs(np.sin(eta)) ** eps1)

        elif SQ_type == 1: 
            eta = np.linspace(-np.pi / 2 - 1, np.pi / 2 - 1, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * np.sign(1 / np.cos(eta)) * (np.abs(1 / np.cos(eta)) ** eps1) * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * np.sign(1 / np.cos(eta)) * (np.abs(1 / np.cos(eta)) ** eps1) * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * np.sign(np.tan(eta)) * (np.abs(np.tan(eta)) ** eps1)

        elif SQ_type == 2: 
            eta = np.linspace(-np.pi, np.pi, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * (np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) + a4) * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * (np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) + a4) * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * np.sign(np.sin(eta)) * (np.abs(np.sin(eta)) ** eps1)

        elif SQ_type == 3: 
            eta = np.linspace(0, 1, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * eta * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * eta * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * ((eta ** (2 / eps1)) - 1)

        else:
            raise ValueError("Incorrect SQ_type input")

        pcl_SQ = np.vstack((X_surf.ravel(), Y_surf.ravel(), Z_surf.ravel())).T

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(angle_x), -np.sin(angle_x)],
            [0, np.sin(angle_x), np.cos(angle_x)]
        ])
        Ry = np.array([
            [np.cos(angle_y), 0, np.sin(angle_y)],
            [0, 1, 0],
            [-np.sin(angle_y), 0, np.cos(angle_y)]
        ])
        Rz = np.array([
            [np.cos(angle_z), -np.sin(angle_z), 0],
            [np.sin(angle_z), np.cos(angle_z), 0],
            [0, 0, 1]
        ])
        R = Rz @ Ry @ Rx 

        pcl_SQ = pcl_SQ @ R.T

        pcl_SQ += np.array([trans_x, trans_y, trans_z])

        return pcl_SQ

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def pcl_dist(self, pcl, SQ):

        reference = pcl  
        sample = SQ 
        distMat = np.zeros(sample.shape[0])

        for row_idx in range(sample.shape[0]):
            diff = reference - sample[row_idx, :]
            D = np.sqrt(np.sum(diff**2, axis=1))
            distMat[row_idx] = np.min(D)

        SQ_pcl = np.mean(distMat)

        reference = SQ  
        sample = pcl  
        distMat = np.zeros(sample.shape[0])

        for row_idx in range(sample.shape[0]):
            diff = reference - sample[row_idx, :]
            D = np.sqrt(np.sum(diff**2, axis=1))
            distMat[row_idx] = np.min(D)

        pcl_SQ = np.mean(distMat)

        return pcl_SQ, SQ_pcl

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def read_point_cloud(self, file_path):
        pcd = o3d.io.read_point_cloud(file_path)
        return np.asarray(pcd.points)
    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def test_superquadric_fitting(self, pcd):
        SQ_type = [0,1,2,3]
        fit_params, fit_type, SQ, residue_SQ = self.SQ_fitting_params(pcd, SQ_type)
        if self.verbose > 0:
            print(pcd.shape)
            print("Fitted Parameters:", fit_params)
            print("Fitted Superquadric Type:", fit_type)
            print("Residual Error:", residue_SQ)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def display_pcd(self, pcd, str=""):
        
        print(f"Number of points: {len(pcd.points)}")
        self.visualize(pcd, str)

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def normalize_y_axis(self, pcd):
        points = np.asarray(pcd.points)
        y_mean = np.mean(points[:, 1])
        points[:, 1] -= y_mean
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

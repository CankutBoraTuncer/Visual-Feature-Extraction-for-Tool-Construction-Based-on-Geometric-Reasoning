import numpy as np
import open3d as o3d
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize, Bounds
import copy
import matplotlib.pyplot as plt

class SQF():

    def __init__(self, pcd, verbose = 0):
        self.pcd = pcd
        self.iteration_count = 0
        self.verbose = verbose

    def fit(self):
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

        return self.test_superquadric_fitting(filtered_points)

    def visualize(self, obj, window_name='Open3D'):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name, width=800, height=600)
        vis.add_geometry(obj)
        vis.run()
        vis.destroy_window()

    def normalize_point_cloud(self, pcd):
        points = np.asarray(pcd.points)
        center = points.mean(axis=0)
        points -= center
        max_abs_value = np.max(np.abs(points))
        points /= max_abs_value
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd

    def set_axes_equal(self, ax):
        """
        Set 3D plot axes to equal scale.

        This ensures the visual proportions of the plot match the actual data.
        """
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d()
        ])
        ranges = np.abs(limits[:, 1] - limits[:, 0])
        max_range = ranges.max()

        # Center the axes around their midpoints
        midpoints = np.mean(limits, axis=1)
        ax.set_xlim3d([midpoints[0] - max_range / 2, midpoints[0] + max_range / 2])
        ax.set_ylim3d([midpoints[1] - max_range / 2, midpoints[1] + max_range / 2])
        ax.set_zlim3d([midpoints[2] - max_range / 2, midpoints[2] + max_range / 2])



    def SQ_fitting_params(self, point_cloud, SQ_type):
        """
        General superquadric fitting to tool parts with constraints and visualization.

        Args:
            point_cloud: Input point cloud (Nx3 array).
            SQ_type: List of superquadric types to fit.

        Returns:
            fit_params: Optimized parameters of the superquadric.
            fit_type: Type of best-fit superquadric.
            SQ: Point cloud representation of the fitted superquadric.
            residue_SQ: Residual error of the fit.
        """
        segment_members = copy.deepcopy(point_cloud)  # Assume point cloud is provided as Nx3 numpy array
        segment_members_original = copy.deepcopy(segment_members)
        residue_SQ = float('inf')

        # Perform PCA and obtain the inverse transformation
        # segment_members, inv_pca = pca_segment(segment_members)

        # recovered_points = segment_members @ inv_pca.T
        # mse = np.mean(np.sum((segment_members_original - recovered_points) ** 2, axis=1))

        # #print(f"Mean Squared Error: {mse}")
        # assert mse < 1e-3, "There is something wrong with PCA"

        for sq_type in SQ_type:
            # Parameter initialization for the segment
            scale, orientations, eps, p, bound_min, bound_max = self.param_init(segment_members, sq_type)
            ##print(f"bbox size: {np.linalg.norm(bound_min-bound_max)}")

            # Define parameter bounds
            lower_bounds = [
                0.8 * scale[0], 0.8 * scale[1], 0.8 * scale[2], 0.8 * scale[3], 0.1, 0.1,
                -np.pi, -np.pi, -np.pi, bound_min[0], bound_min[1], bound_min[2]
            ]
            upper_bounds = [
                1.2 * scale[0], 1.2 * scale[1], 1.2 * scale[2], 1.2 * scale[3], 2.0, 2.0,
                np.pi, np.pi, np.pi, bound_max[0], bound_max[1], bound_max[2]
            ]
            bounds = Bounds(lower_bounds, upper_bounds)

            # Initial parameter guess
            x_init = np.array([
                scale[0], scale[1], scale[2], scale[3], eps[0], eps[1],
                orientations[0], orientations[1], orientations[2],
                p[0], p[1], p[2]
            ])


            x_init = [float(x) for x in x_init]
            x_init[4:] = [x + 1e-6 if abs(x) < 1e-6 else x for x in x_init[4:]]

            if self.verbose > 0:
                for i, (x, lb, ub) in enumerate(zip(x_init, lower_bounds, upper_bounds)):
                    if not (lb <= x <= ub):
                        print(f"Parameter {i} out of bounds: {x} not in [{lb}, {ub}]")


            #print(f"x_init: {x_init}")
            #print(f"lowe_bound: {lower_bounds}")
            #print(f"upper_bounds: {upper_bounds}")

            # Objective function for optimization
            def objective_fn(params):
                cost = self.fitting_fn(params, segment_members, sq_type)
                return np.sum(cost)

            # Perform optimization using only bounds
            result = minimize(
                objective_fn,
                x0=x_init,
                bounds=bounds,
                method='SLSQP',
                options={'disp': False, 'maxiter': 5000, 'ftol': 1e-8}
            )

            # Check optimization success
            if not result.success:
                raise ValueError(f"Optimization failed: {result.message}")

            # Extract the optimized parameters
            optimum_quadrics = result.x

            #print(f"optimum_quadrics: {optimum_quadrics}")

            # Generate the superquadric point cloud
            SQ = self.SQ2PCL(optimum_quadrics, sq_type)
            # SQ = np.dot(SQ, inv_pca.T)

            # Calculate the residual
            pcl_SQ_dist, SQ_pcl_dist = self.pcl_dist(segment_members_original, SQ)
            residue = pcl_SQ_dist + SQ_pcl_dist
            #print(f"residue is for type {sq_type} is {residue}")

            if residue < residue_SQ:
                SQ_optimum = optimum_quadrics
                residue_SQ = residue
                optimum_type = sq_type


        # Generate the final superquadric point cloud
        #print(f"SQ_optimum: {SQ_optimum}")
        # SQ_optimum = np.array([0.0051, 0.0143, 0.0757, 0.0000, 0.1000, 0.7779, -1.2058, 1.5672, 0.0096, 0.0409, -0.0025, 0.9990])
        # 0.0051    0.0143    0.0757    0.0000    0.1000    0.7779   -1.2058    1.5672    0.0096    0.0409   -0.0025    0.9990
        SQ = self.SQ2PCL(SQ_optimum, optimum_type)
        # SQ = np.dot(SQ, inv_pca.T)

        #print("mean of the point cloud: ", np.mean(SQ, axis=0))


        # fit_params = copy.deepcopy(SQ_optimum)
        # # Extract parameters for the best-fit superquadric
        # scale, orientations, eps, p, _, _ = param_init(SQ, optimum_type)

        if optimum_type == 2:  # Toroid
            fit_params = copy.deepcopy(SQ_optimum)
            # fit_params = np.zeros(12)
            # fit_params[:4] = scale
            # fit_params[4:6] = eps
            # fit_params[6:9] = orientations
            # fit_params[9:12] = p
        else:
            fit_params = np.zeros(11)
            fit_params[:3] = SQ_optimum[:3]
            fit_params[3:5] = SQ_optimum[4:6]
            fit_params[5:8] = SQ_optimum[6:9]
            fit_params[8:11] = SQ_optimum[9:12]

        fit_type = optimum_type

    # Combined plot: original point cloud and fitted superquadric

        # rotate around x  and y axis 90 degree
        if self.verbose > 0:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            #print(f"Number of points in the point cloud: {len(segment_members_original)}")
            # Plot original point cloud
            ax.scatter(
                segment_members_original[:, 0],
                segment_members_original[:, 1],
                segment_members_original[:, 2],
                s=3,
                label="Original Point Cloud",
                alpha=0.6,
                c="blue"
            )

            # Plot fitted superquadric
            ax.scatter(
                SQ[:, 0],
                SQ[:, 1],
                SQ[:, 2],
                s=3,
                label="Fitted Superquadric",
                alpha=0.6,
                c="red"
            )

            # Set plot details
            ax.set_title("Original Point Cloud and Fitted Superquadric")
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_zlabel("Z-axis")
            ax.legend()
            self.set_axes_equal(ax)
            # Show the plot
            plt.show()

        # points_original = SQ[:, :3]  # Ensure the correct slicing
        # temp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points_original))
        # # visualize(temp, "fitted superquadric")
        # points_original = segment_members_original[:, :3]  # Ensure the correct slicing
        # temp2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points_original))
        # visualize_3d_open3d(temp2, temp)
        # visualize(temp2, "original point cloud")


        # # read point cloud from .ply file and display with calculated superquadric
        # matlab_pcd = o3d.io.read_point_cloud("/Users/hsimsir/Documents/MATLAB/fitted_superquadric.ply")
        # matlab_pcd_pts = np.asarray(matlab_pcd.points)
        # #print("Number of points in the point cloud: ", len(matlab_pcd_pts))
        # fig = plt.figure(figsize=(10, 8))
        # ax = fig.add_subplot(111, projection='3d')

        # # Plot original point cloud
        # ax.scatter(
        #     matlab_pcd_pts[:, 0],
        #     matlab_pcd_pts[:, 1],
        #     matlab_pcd_pts[:, 2],
        #     s=1,
        #     label="matlab Point Cloud",
        #     alpha=0.6,
        #     c="blue"
        # )

        # # Plot fitted superquadric
        # ax.scatter(
        #     SQ[:, 0],
        #     SQ[:, 1],
        #     SQ[:, 2],
        #     s=1,
        #     label="Fitted Superquadric",
        #     alpha=0.6,
        #     c="red"
        # )

        # # Set plot details
        # ax.set_title("Original Point Cloud and Fitted Superquadric")
        # ax.set_xlabel("X-axis")
        # ax.set_ylabel("Y-axis")
        # ax.set_zlabel("Z-axis")
        # ax.legend()

        # # Show the plot
        # plt.show()

        return fit_params, fit_type, SQ, residue_SQ


    def pca_segment(self, pcl):
        """
        Compute PCA for point cloud (PCL) segments to get segment orientation.

        Parameters:
        - pcl: numpy.ndarray of shape (n_points, 3), the point cloud segment.

        Returns:
        - new_segments: numpy.ndarray, the segment projected using PCA.
        - inv_pca: numpy.ndarray, the inverse of the PCA transformation matrix.
        """
        # Ensure input is a NumPy array
        segments = np.array(pcl)

        # Optional downsampling can be implemented if needed here.

        # Perform PCA
        pca = PCA(n_components=3)  # We expect a 3D point cloud
        pca.fit(segments)          # Fit PCA to the data

        # Transformation matrix (principal components)
        pca_segments = pca.components_.T  # PCA components as columns

        # Project original points into PCA space
        new_segments = segments @ pca_segments

        # Compute the inverse PCA transformation matrix
        inv_pca = np.linalg.inv(pca_segments)

        return new_segments, inv_pca

    def param_init(self, segment, sq_type):
        """
        Initialize parameters for the superquadric fitting.

        Args:
            segment (numpy.ndarray): Nx3 array of 3D points belonging to a segment of the original point cloud.
            sq_type (int): Type of the superquadric (e.g., 2 for toroids, others for general superquadrics).

        Returns:
            scale (numpy.ndarray): Scale parameters (size along principal axes).
            orientations (numpy.ndarray): Orientation angles (Euler angles).
            eps (list): Exponential factors for the superquadric.
            p (numpy.ndarray): Mean position of the segment.
            bound_min (numpy.ndarray): Minimum bounds of the segment.
            bound_max (numpy.ndarray): Maximum bounds of the segment.
        """
        scale = np.zeros(4)
        segment_members = segment

        # Perform PCA on the segment
        pca = PCA(n_components=3)
        pca.fit(segment_members)
        rotation_matrix = pca.components_
        orientations = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)

        # Find bounding boxes
        minimum = np.min(segment_members, axis=0)
        maximum = np.max(segment_members, axis=0)

        bound_min = minimum
        bound_max = maximum

        # Compute scale and ensure no dimension is zero
        pcl_scale = np.abs(maximum - minimum)
        pcl_scale = np.where(pcl_scale == 0, pcl_scale + 0.000005, pcl_scale)

        eps = [0.1, 1.0]  # Fixed epsilon values for superellipsoid = cylinder

        # Adjust scales and orientations if conditions are met
        if sq_type != 3 and pcl_scale[2] > 0.25 * pcl_scale[1]:
            z_scale = pcl_scale[0]
            pcl_scale[0] = pcl_scale[2]
            pcl_scale[2] = z_scale
            orientations[1] += np.pi / 2  # Adjust pitch

        scale[:3] = pcl_scale / 2  # Divide by 2 as scale represents radius/half-extent

        if sq_type == 2:  # For toroids
            scale[3] = 0.02  # Additional scale parameter for toroids

        # Compute the mean position
        p = np.mean(segment_members, axis=0)

        return scale, orientations, eps, p, bound_min, bound_max


    iteration_count = 0  # Global variable to track iteration count

    def param_init(self, segment, sq_type):
        """
        Initialize parameters for the superquadric fitting.

        Args:
            segment (numpy.ndarray): Nx3 array of 3D points belonging to a segment of the original point cloud.
            sq_type (int): Type of the superquadric (e.g., 2 for toroids, others for general superquadrics).

        Returns:
            scale (numpy.ndarray): Scale parameters (size along principal axes).
            orientations (numpy.ndarray): Orientation angles (Euler angles).
            eps (list): Exponential factors for the superquadric.
            p (numpy.ndarray): Mean position of the segment.
            bound_min (numpy.ndarray): Minimum bounds of the segment.
            bound_max (numpy.ndarray): Maximum bounds of the segment.
        """
        scale = np.zeros(4)
        segment_members = segment

        # Perform PCA on the segment
        pca = PCA(n_components=3)
        pca.fit(segment_members)
        rotation_matrix = pca.components_
        orientations = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)

        # Find bounding boxes
        minimum = np.min(segment_members, axis=0)
        maximum = np.max(segment_members, axis=0)

        bound_min = minimum
        bound_max = maximum

        # Compute scale and ensure no dimension is zero
        pcl_scale = np.abs(maximum - minimum)
        pcl_scale = np.where(pcl_scale == 0, pcl_scale + 0.000005, pcl_scale)

        eps = [0.1, 1.0]  # Fixed epsilon values for superellipsoid = cylinder

        # Adjust scales and orientations if conditions are met
        if sq_type != 3 and pcl_scale[2] > 0.25 * pcl_scale[1]:
            z_scale = pcl_scale[0]
            pcl_scale[0] = pcl_scale[2]
            pcl_scale[2] = z_scale
            orientations[1] += np.pi / 2  # Adjust pitch

        scale[:3] = pcl_scale / 2  # Divide by 2 as scale represents radius/half-extent

        if sq_type == 2:  # For toroids
            scale[3] = 0.02  # Additional scale parameter for toroids

        # Compute the mean position
        p = np.mean(segment_members, axis=0)

        return scale, orientations, eps, p, bound_min, bound_max


      # Global variable to track iteration count

    def fitting_fn(self, opt_params, current_segment, SQ_type):
        """
        Fitting function for superquadrics.

        Args:
            opt_params (numpy.ndarray): Parameters for optimization (12 elements).
            current_segment (numpy.ndarray): Nx3 array of 3D points in the current segment.
            SQ_type (int): Type of the superquadric.

        Returns:
            numpy.ndarray: The cost for the optimization process.
        """
        self.iteration_count += 1

        pcl = current_segment

        # Scaling parameters
        a1, a2, a3, a4 = opt_params[:4]

        # Exponential terms
        eps1, eps2 = opt_params[4:6]
        eps1 = max(eps1, 1e-6)  # Avoid zero or negative eps1
        eps2 = max(eps2, 1e-6)  # Avoid zero or negative eps2

        # Rotation angles
        angle_x, angle_y, angle_z = opt_params[6:9]

        # Translation parameters
        trans_x, trans_y, trans_z = opt_params[9:12]

        # Rotation matrix components
        nx = np.cos(angle_x) * np.cos(angle_y) * np.cos(angle_z) - (np.sin(angle_x) * np.sin(angle_z))
        ny = np.sin(angle_x) * np.cos(angle_y) * np.cos(angle_z) + np.cos(angle_x) * np.sin(angle_z)
        nz = -(np.sin(angle_y) * np.cos(angle_z))
        ox = -(np.cos(angle_x) * np.cos(angle_y) * np.sin(angle_z)) - (np.sin(angle_x) * np.cos(angle_z))
        oy = -(np.sin(angle_x) * np.cos(angle_y) * np.sin(angle_z)) + np.cos(angle_x) * np.cos(angle_z)
        oz = np.sin(angle_y) * np.sin(angle_z)
        ax = np.cos(angle_x) * np.sin(angle_y)
        ay = np.sin(angle_x) * np.sin(angle_y)
        az = np.cos(angle_y)

        # Transform the point cloud
        X = (nx * pcl[:, 0] + ny * pcl[:, 1] + nz * pcl[:, 2] - trans_x * nx - trans_y * ny - trans_z * nz)
        Y = (ox * pcl[:, 0] + oy * pcl[:, 1] + oz * pcl[:, 2] - trans_x * ox - trans_y * oy - trans_z * oz)
        Z = (ax * pcl[:, 0] + ay * pcl[:, 1] + az * pcl[:, 2] - trans_x * ax - trans_y * ay - trans_z * az)

        # Scale the ellipsoid
        a1 = max(a1, 1e-6)
        a2 = max(a2, 1e-6)
        a3 = max(a3, 1e-6)

        X = np.abs(X) / a1
        Y = np.abs(Y) / a2
        Z = np.abs(Z) / a3

        X1 = np.abs(X ** (2 / eps2))
        Y1 = np.abs(Y ** (2 / eps2))
        Z1 = np.abs(Z ** (2 / eps1))
        # Inside-outside function based on SQ_type
        if SQ_type in [0, 1]:
            quadric_func = (X1 + Y1) ** (eps2 / eps1) + Z1
        elif SQ_type == 2:
            quadric_func = np.abs(((X1 + Y1) ** (eps2 / 2) - a4)) ** (2 / eps1) + Z1
        elif SQ_type == 3:
            quadric_func = ((X1 + Y1) ** (eps2 / eps1)) - Z
        else:
            raise ValueError("Incorrect SQ_type input")


        quadric_func = np.abs(quadric_func)

        cost = (a1 * a2 * a3) * (np.abs(quadric_func ** eps1) - 1) ** 2

        cost = np.nan_to_num(cost, nan=0.0)  # Replace NaN with 0.0


        # Display iteration and cost
        # #print(f"Iteration: {iteration_count}, Cost: {np.sum(cost)}")

        return cost


    def SQ2PCL(self, params, SQ_type):
        """
        Function to generate the superquadric as a point cloud.

        Args:
            params (list or np.ndarray): Optimized parameters for the superquadric.
            SQ_type (int): Type of the superquadric (0: superellipsoid, 1: hyperboloid, 2: toroid, 3: paraboloid).

        Returns:
            np.ndarray: Point cloud segments corresponding to the superquadric.
        """
        n = 50  # Number of points in the superquadric point cloud

        # Extract parameters
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

        # Rotation and translation parameters
        angle_x = params[-6]
        angle_y = params[-5]
        angle_z = params[-4]
        trans_x = params[-3]
        trans_y = params[-2]
        trans_z = params[-1]  # Corrected indexing

        # Generate the surface points for the superquadric based on its type
        if SQ_type == 0:  # Superellipsoid
            eta = np.linspace(-np.pi / 2, np.pi / 2, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * np.sign(np.sin(eta)) * (np.abs(np.sin(eta)) ** eps1)

        elif SQ_type == 1:  # Hyperboloid
            eta = np.linspace(-np.pi / 2 - 1, np.pi / 2 - 1, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * np.sign(1 / np.cos(eta)) * (np.abs(1 / np.cos(eta)) ** eps1) * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * np.sign(1 / np.cos(eta)) * (np.abs(1 / np.cos(eta)) ** eps1) * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * np.sign(np.tan(eta)) * (np.abs(np.tan(eta)) ** eps1)

        elif SQ_type == 2:  # Toroid
            eta = np.linspace(-np.pi, np.pi, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * (np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) + a4) * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * (np.sign(np.cos(eta)) * (np.abs(np.cos(eta)) ** eps1) + a4) * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * np.sign(np.sin(eta)) * (np.abs(np.sin(eta)) ** eps1)

        elif SQ_type == 3:  # Paraboloid
            eta = np.linspace(0, 1, n)
            omega = np.linspace(-np.pi, np.pi, n)
            eta, omega = np.meshgrid(eta, omega)

            X_surf = a1 * eta * np.sign(np.cos(omega)) * (np.abs(np.cos(omega)) ** eps2)
            Y_surf = a2 * eta * np.sign(np.sin(omega)) * (np.abs(np.sin(omega)) ** eps2)
            Z_surf = a3 * ((eta ** (2 / eps1)) - 1)

        else:
            raise ValueError("Incorrect SQ_type input")

        # Combine the surface points into a single point cloud
        pcl_SQ = np.vstack((X_surf.ravel(), Y_surf.ravel(), Z_surf.ravel())).T

        # Debugging: Verify the initial mean before transformations
        #print(f"Initial mean of pcl_SQ before rotation/translation: {np.mean(pcl_SQ, axis=0)}")

        # Rotation matrix for angles
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
        R = Rz @ Ry @ Rx # Combined rotation matrix

        # Apply rotation
        pcl_SQ = pcl_SQ @ R.T

        # Debugging: Check the mean after rotation
        #print(f"Mean of pcl_SQ after rotation: {np.mean(pcl_SQ, axis=0)}")

        # Apply translation
        pcl_SQ += np.array([trans_x, trans_y, trans_z])

        # Debugging: Check the final mean after translation
        #print(f"Final mean of pcl_SQ after translation: {np.mean(pcl_SQ, axis=0)}")

        return pcl_SQ

    def pcl_dist(self, pcl, SQ):
        """
        Computes the mean distance between a point cloud and a superquadric.

        Args:
            pcl (numpy.ndarray): Nx3 array representing the point cloud.
            SQ (numpy.ndarray): Mx3 array representing the superquadric point cloud.

        Returns:
            pcl_SQ (float): Mean distance from the point cloud to the superquadric.
            SQ_pcl (float): Mean distance from the superquadric to the point cloud.
        """
        # Distance from SQ to pcl
        reference = pcl  # Points in the original point cloud
        sample = SQ  # Points in the superquadric
        distMat = np.zeros(sample.shape[0])

        for row_idx in range(sample.shape[0]):
            diff = reference - sample[row_idx, :]
            D = np.sqrt(np.sum(diff**2, axis=1))
            distMat[row_idx] = np.min(D)

        SQ_pcl = np.mean(distMat)

        # Distance from pcl to SQ
        reference = SQ  # Now the superquadric is the reference
        sample = pcl  # And the point cloud is the sample
        distMat = np.zeros(sample.shape[0])

        for row_idx in range(sample.shape[0]):
            diff = reference - sample[row_idx, :]
            D = np.sqrt(np.sum(diff**2, axis=1))
            distMat[row_idx] = np.min(D)

        pcl_SQ = np.mean(distMat)

        return pcl_SQ, SQ_pcl


    # Read the point cloud from file
    def read_point_cloud(self, file_path):
        """
        Reads a point cloud from a .ply file using Open3D.
        Args:
            file_path (str): Path to the point cloud file.
        Returns:
            numpy.ndarray: Nx3 array of point cloud points.
        """
        pcd = o3d.io.read_point_cloud(file_path)
        return np.asarray(pcd.points)

    # Simulate SQ fitting (requires the previously defined SQ_fitting_params and helper functions)
    def test_superquadric_fitting(self, pcd):

        # Define a superquadric type for testing
        SQ_type = [0,1,2,3]

        # Perform superquadric fitting
        fit_params, fit_type, SQ, residue_SQ = self.SQ_fitting_params(pcd, SQ_type)
        #print(pcd.shape)
        #print("Fitted Parameters:", fit_params)
        #print("Fitted Superquadric Type:", fit_type)
        #print("Residual Error:", residue_SQ)
        return fit_params
    
    def display_pcd(self, pcd, str):
        #print(f"Number of points: {len(pcd.points)}")
        self.visualize(pcd, str)

    # Specify the point cloud file path

    def normalize_y_axis(self, pcd):
        points = np.asarray(pcd.points)
        y_mean = np.mean(points[:, 1])
        points[:, 1] -= y_mean
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd





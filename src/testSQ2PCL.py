import numpy as np
import matplotlib.pyplot as plt
def SQ2PCL(params, SQ_type):
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
    print(f"Initial mean of pcl_SQ before rotation/translation: {np.mean(pcl_SQ, axis=0)}")

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
    R = Rz @ Ry @ Rx  # Combined rotation matrix

    # Apply rotation
    pcl_SQ = pcl_SQ @ R.T

    # Debugging: Check the mean after rotation
    print(f"Mean of pcl_SQ after rotation: {np.mean(pcl_SQ, axis=0)}")

    # Apply translation
    pcl_SQ += np.array([trans_x, trans_y, trans_z])

    # Debugging: Check the final mean after translation
    print(f"Final mean of pcl_SQ after translation: {np.mean(pcl_SQ, axis=0)}")

    return pcl_SQ
def read_point_cloud_from_txt(file_path):
    """
    Reads a point cloud from a text file where each line contains 3D coordinates (x, y, z).
    
    Args:
        file_path (str): Path to the text file containing the point cloud data.
    
    Returns:
        numpy.ndarray: The point cloud data as an Nx3 NumPy array.
    """
    return np.loadtxt(file_path, delimiter=' ')

def display_two_point_clouds(pcl1, pcl2):
    """
    Displays two point clouds in a single 3D scatter plot.
    
    Args:
        pcl1 (numpy.ndarray): First point cloud as an Nx3 NumPy array.
        pcl2 (numpy.ndarray): Second point cloud as an Nx3 NumPy array.
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot for the first point cloud
    ax.scatter(
        pcl1[:, 0], pcl1[:, 1], pcl1[:, 2],
        s=1, alpha=0.6, label="Matlab point cloud ", c="blue"
    )

    # Scatter plot for the second point cloud
    ax.scatter(
        pcl2[:, 0], pcl2[:, 1], pcl2[:, 2],
        s=1, alpha=0.6, label="Calculated point cloud", c="red"
    )

    # Set plot title and axes labels
    ax.set_title("Comparison of Two Point Clouds", fontsize=16)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.set_zlabel("Z", fontsize=12)

    # Add legend/
    ax.legend()

    # Show grid
    ax.grid(True)

    # Display the plot
    plt.show()

if __name__ == "__main__":
    # File path to the point cloud file
    file_path = "/Users/hsimsir/Documents/MATLAB/point_cloud.txt"
    
    try:
        # Read the first point cloud from the file
        pcl1 = read_point_cloud_from_txt(file_path)
        print(f"Loaded first point cloud with {pcl1.shape[0]} points.")

        # Parameters for the SQ2PCL function
        params = np.array([
            1.28886656e-02, 2.06685175e-02, 6.83940952e-02, 2.62942738e-16,
            1.00000000e-01, 1.00000000e-01, 1.01436982e-01, 1.50867874e+00,
            -7.92529153e-05, 8.08553949e-02, 9.05564416e-03, 1.00281462e+00
        ])
        sq_type = 0

        # Generate the second point cloud using the SQ2PCL function
        pcl2 = SQ2PCL(params, sq_type)
        print(f"Generated second point cloud with {pcl2.shape[0]} points.")

        # Display the two point clouds together
        display_two_point_clouds(pcl1, pcl2)
    except Exception as e:
        print(f"Error: {e}")

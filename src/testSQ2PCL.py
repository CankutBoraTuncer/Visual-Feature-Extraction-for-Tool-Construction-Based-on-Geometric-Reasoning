import numpy as np
import matplotlib.pyplot as plt
import SQF

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
        sq_type = 2

        # Generate the second point cloud using the SQ2PCL function
        pcl2 = SQF.SQ2PCL(params, sq_type)
        print(f"Generated second point cloud with {pcl2.shape[0]} points.")

        # Display the two point clouds together
        display_two_point_clouds(pcl1, pcl2)
    except Exception as e:
        print(f"Error: {e}")

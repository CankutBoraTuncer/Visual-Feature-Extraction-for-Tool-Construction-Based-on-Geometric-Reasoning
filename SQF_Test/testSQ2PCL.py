import numpy as np
import matplotlib.pyplot as plt
import SQF
import open3d as o3d

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
    file_path = "/Users/hsimsir/Documents/CS554_Computer_Vision_Project/src/point_clouds_ref/torus.ply"
    
    pcl = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcl.points)
    print(f"first point cloud with {points.shape[0]} points.")

    # Parameters for the SQ2PCL function
    params = np.array([
        0.1914, 0.4574, 0.4107, 0.1000,
        1.0000, 0.0004, 1.5749, 0.0880,
        0.5195, 0.0010, 0.0033
    ])

    sq_type = 0

    # Generate the second point cloud using the SQ2PCL function
    points2 = SQF.SQ2PCL(params, sq_type)
    print(f"Generated second point cloud with {points2.shape[0]} points.")
    SQF.visualizePointClouds([points, points2], labels=['pcl', 'pcl2'])

    # Display the two point clouds together
    # display_two_point_clouds(pcl1, pcl2)
    pcl_SQ, SQ_pcl = SQF.pcl_dist(points, points2)
    print(f"residue is: {pcl_SQ + SQ_pcl}")

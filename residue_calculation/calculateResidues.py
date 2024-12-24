import os
import open3d as o3d
import SQF
import numpy as np
from sklearn.decomposition import PCA


# Superquadric type
SQ_type = [3] 

# Define the directory containing point clouds
folder_path = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/src/point_clouds_ref/'
output_file = f'/Users/hsimsir/Documents/CS554_Computer_Vision_Project/our_residue_results_ref_{SQ_type[0]}.txt'

# Check if the directory exists
if not os.path.isdir(folder_path):
    raise FileNotFoundError(f'The specified directory does not exist: {folder_path}')

# Get all PLY files in the directory
ply_files = [f for f in os.listdir(folder_path) if f.endswith('.ply')]

# Open a file for writing the results
with open(output_file, 'w') as file:
    # Write header to the file
    file.write('Point Cloud Name, Residue\n')

    # Process each point cloud file
    for ply_file in ply_files:
        file_path = os.path.join(folder_path, ply_file)
        print(f"file path: {file_path}")

        # Read the point cloud
        point_cloud = o3d.io.read_point_cloud(file_path)
        points = np.asarray(point_cloud.points)
        
        # pca = PCA(n_components=3)
        # pca.fit(points)
        # rotation_matrix = pca.components_
        # points = points @ rotation_matrix.T

        # point_cloud.points = o3d.utility.Vector3dVector(points)
        # SQF.normalize_point_cloud(point_cloud)
        # points = np.asarray(point_cloud.points)

        # Debug log for point cloud processing
        print(f'Processing: {ply_file}')

        # Perform superquadric fitting
        try: 
            result = SQF.SQ_fitting_params(points, SQ_type)
        except ValueError as ve:
            print(f'ValueError processing {ply_file}: {ve}')
            continue
            

        # Debug log for function return value
        print(f'SQ_fitting_params returned: {result}')

        # Ensure the result is a tuple or list with the expected size
        if not isinstance(result, (tuple, list)) or len(result) != 4:
            raise ValueError(f"Unexpected return value from SQ_fitting_params: {result}")

        # Unpack the result
        fitting_params, _, fitting_pcl, residue = result

        # Write results to the file
        file.write(f'{ply_file}, {residue:.4f}\n')

        # Display the residue and fitting parameters
        print(f'Processed: {ply_file}')
        print(f'Residue: {residue:.4f}')
        print(f'Fitting Params: {fitting_params}')

print(f'Residue calculation complete. Results written to {output_file}')

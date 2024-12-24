% Define the directory containing point clouds
folder_path = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/src/point_clouds_aux';
output_file = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/residue_results_aux_2.txt';

% Check if the directory exists
if ~isfolder(folder_path)
    error('The specified directory does not exist: %s', folder_path);
end

% Get all PLY files in the directory
ply_files = dir(fullfile(folder_path, '*.ply'));

% Open a file for writing the results
fileID = fopen(output_file, 'w');
if fileID == -1
    error('Could not open the file for writing: %s', output_file);
end

% Write header to the file
fprintf(fileID, 'Point Cloud Name, Residue\n');

% Superquadric type
SQ_type = 2; % Example type for superellipsoid

% Process each point cloud file
for i = 1:length(ply_files)
    try
        % Get the file path
        file_path = fullfile(folder_path, ply_files(i).name);

        % Read the point cloud
        point_cloud = pcread(file_path);

        % Normalize the point cloud
        point_cloud_temp = pcl_tf(point_cloud);

        % Perform superquadric fitting
        [fitting_params, ~, fitting_pcl, residue] = SQ_fitting_params(point_cloud_temp, SQ_type);

        % Write results to the file
        fprintf(fileID, '%s, %.4f\n', ply_files(i).name, residue);

        % % Optionally save the fitted superquadric
        % fitted_output_path = fullfile(folder_path, ['fitted_', ply_files(i).name]);
        % pcwrite(fitting_pcl, fitted_output_path);

        % Display the residue and fitting parameters
        fprintf('Processed: %s\n', ply_files(i).name);
        fprintf('Residue: %.4f\n', residue);
        fprintf('Fitting Params: %s\n', mat2str(fitting_params));

    catch ME
        fprintf('Error processing %s: %s\n', ply_files(i).name, ME.message);
    end
end

% Close the results file
fclose(fileID);

fprintf('Residue calculation complete. Results written to %s\n', output_file);

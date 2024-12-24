% process_point_cloud.m
% This script reads a point cloud from a .ply file, normalizes its orientation
% using the pcl_tf function, and calls the SQ_fitting_params function.

% Clear workspace and command window
clear; clc;

%% === User Inputs ===
% Specify the path to your .ply file
plyFile = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/src/point_clouds_ref/hammer_large.ply'; % <-- Replace with your .ply file path

% Specify the SQ_type to be used in SQ_fitting_params
SQ_type = 0; % <-- Replace with the desired SQ_type (e.g., 'type1')

%% === Validate Inputs ===
% Check if the specified .ply file exists
if ~isfile(plyFile)
    error('The specified .ply file does not exist: %s', plyFile);
end

% Check if the Computer Vision Toolbox is available
if ~license('test', 'video_and_image_blockset') && ~license('test', 'computer_vision_toolbox')
    error('Computer Vision Toolbox is required to read .ply files.');
end

% Check if pcl_tf function exists
if ~exist('pcl_tf', 'file')
    error('The function pcl_tf is not found. Ensure it is in the MATLAB path.');
end

% Check if SQ_fitting_params function exists
if ~exist('SQ_fitting_params', 'file')
    error('The function SQ_fitting_params is not found. Ensure it is in the MATLAB path.');
end

%% === Read the Point Cloud ===
try
    ptCloud = pcread(plyFile);
    fprintf('Successfully read point cloud from %s\n', plyFile);
catch ME
    error('Error reading the .ply file: %s', ME.message);
end

% Extract point locations (Nx3 matrix)
points = ptCloud.Location;

%% === Normalize Point Cloud Orientation ===
% Assuming pcl_tf takes the point cloud as input and returns the transformed point cloud
try
    % The variable name 'input_tool_action' is used as per user instruction.
    % Ensure that 'input_tool_action' is defined appropriately.
    input_tool_action = points; % Pass the raw points to pcl_tf
    normalized_point_cloud = pcl_tf(input_tool_action);
    fprintf('Point cloud normalized using pcl_tf function.\n');
catch ME
    error('Error during point cloud normalization with pcl_tf: %s', ME.message);
end

% Verify that normalized_point_cloud is Nx3
if size(normalized_point_cloud, 2) ~= 3
    error('Normalized point cloud must be an Nx3 matrix.');
end

%% === Call the SQ_fitting_params Function ===
try
    [fit_params, fit_type, SQ, residue_SQ] = SQ_fitting_params(normalized_point_cloud, SQ_type);
    fprintf('SQ_fitting_params function executed successfully.\n');
catch ME
    error('Error calling SQ_fitting_params: %s', ME.message);
end

%% === Display Results ===
disp('--- Fitting Parameters ---');
disp(fit_params);

disp('--- Fit Type ---');
disp(fit_type);

disp('--- SQ ---');
disp(SQ);

disp('--- Residue SQ ---');
disp(residue_SQ);

%% === (Optional) Visualize the Normalized Point Cloud ===
% Uncomment the following lines to visualize the normalized point cloud
% figure;
% pcshow(normalized_point_cloud);
% title('Normalized Point Cloud');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

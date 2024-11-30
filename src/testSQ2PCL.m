% Add the directory containing SQ2PCL.m to the MATLAB path
addpath('/Users/hsimsir/Documents/Robogyver-Tool-Macgyvering/auxiliary/MATLAB_code/'); % Replace 'path/to/SQ2PCL' with the actual directory

% Define input parameters
params = [1.28886656e-02, 2.06685175e-02, 6.83940952e-02, 2.62942738e-16, ...
          1.00000000e-01, 1.00000000e-01, 1.01436982e-01, 1.50867874e+00, ...
         -7.92529153e-05, 8.08553949e-02, 9.05564416e-03, 1.00281462e+00];
SQ_type = 0; % Superquadric type

% Call the SQ2PCL function
pcl_segments = SQ2PCL(params, SQ_type);

% Save the point cloud to a file
file_name = 'point_cloud.txt';
writematrix(pcl_segments, file_name, 'Delimiter', ' ');

% Confirm the save
disp(['Point cloud saved to ', file_name]);

% Remove the added path if not needed afterward
rmpath('/Users/hsimsir/Documents/Robogyver-Tool-Macgyvering/auxiliary/MATLAB_code');

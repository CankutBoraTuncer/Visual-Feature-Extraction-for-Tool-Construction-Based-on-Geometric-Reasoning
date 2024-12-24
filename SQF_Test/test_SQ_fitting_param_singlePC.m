% Superquadric type
SQ_type = 0;

file_path = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/output_point_cloud_4.ply';
output_path = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/ml_output_point_cloud_4.ply';
disp(file_path)
point_cloud = pcread(file_path);
% pcd_ml = pcread(output_path);

% Normalize the point cloud
point_cloud_temp = pcl_tf(point_cloud);

% Perform superquadric fitting
[fitting_params, ~, fitting_pcl, residue] = SQ_fitting_params(point_cloud_temp, SQ_type);

pcwrite(fitting_pcl, output_path);

% % Optionally save the fitted superquadric
% fitted_output_path = fullfile(folder_path, ['fitted_', ply_files(i).name]);
% pcwrite(fitting_pcl, fitted_output_path);

% Display the residue and fitting parameters
fprintf('Residue: %.4f\n', residue);
fprintf('Fitting Params: %s\n', mat2str(fitting_params));

figure;
pcshowpair(point_cloud, pcd_ml);
title(sprintf('Fitting ellipsoid to PC. Residue: %d', residue));
xlabel('X');
ylabel('Y');
zlabel('Z');

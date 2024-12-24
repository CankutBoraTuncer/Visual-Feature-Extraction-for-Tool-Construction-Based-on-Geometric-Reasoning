% Define the file path
file_path = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/edited_pcl.ply';

% Check if the file exists
if ~isfile(file_path)
    error('The specified file does not exist: %s', file_path);
end

% Read the point cloud from the file
point_cloud = pcread(file_path);
segment_members = point_cloud.Location; % Extract Nx3 point array

% Superquadric type
SQ_type = 2; % Example type for superellipsoid

% Initial parameters (12 parameters)
x_init = [
    0.06161033189955391, 0.11922311476483836, 0.5097239528361709, 0.02, ...
    0.1, 1.0, 0.015842171469100705, 1.5717786304102472, -0.008135844327065224, ...
    0.5101328209876588, -0.0007222597651816321, -0.0005506903443385312
];

% Minimum bounds for the parameters
lower_bound = [
    0.04928826551964313, 0.09537849181187069, 0.40777916226893673, 0.006, ...
    0.1, 0.1, -pi, -pi, -pi, ...
    -0.019447905672341687, -0.11983178097185217, -0.06154851318302501
];

% Maximum bounds for the parameters
upper_bounds = [
    0.07393239827946468, 0.14306773771780604, 0.611668743403405, 0.034, ...
    2.0, 2.0, pi, pi, pi, ...
    1.0, 0.11861444855782456, 0.06167215061608281
];


% Optimization
options = optimset('Display', 'iter', 'TolX', 1e-10, 'TolFun', 1e-10, 'MaxIter', 3000, 'MaxFunEvals', 3000);
[optimum_quadrics, ~, ~, ~, ~] = lsqnonlin(@(x) fitting_fn(x, segment_members, SQ_type), x_init, lower_bound, upper_bounds, options);

% Display the optimized parameters
disp('Optimized Quadrics Parameters:');
disp(optimum_quadrics);

% Extract and display individual parameters for clarity
fprintf('Fitted Superquadric Parameters:\n');
fprintf('  a1 (Scale X): %.6f\n', optimum_quadrics(1));
fprintf('  a2 (Scale Y): %.6f\n', optimum_quadrics(2));
fprintf('  a3 (Scale Z): %.6f\n', optimum_quadrics(3));
fprintf('  eps1 (Shape Parameter 1): %.6f\n', optimum_quadrics(5));
fprintf('  eps2 (Shape Parameter 2): %.6f\n', optimum_quadrics(6));
fprintf('  Rotations (Euler Angles): [%.6f, %.6f, %.6f]\n', optimum_quadrics(7:9));
fprintf('  Translations: [%.6f, %.6f, %.6f]\n', optimum_quadrics(10:12));

% Generate the fitted superquadric and display it
fitted_SQ = SQ2PCL(optimum_quadrics, SQ_type); % Assume this function is defined

% Display the fitted superquadric point cloud
pcshow(fitted_SQ, 'MarkerSize', 50);
title('Fitted Superquadric');

fitted_SQ_pointCloud = pointCloud(fitted_SQ);
pcwrite(fitted_SQ_pointCloud, 'fitted_superquadric.ply');
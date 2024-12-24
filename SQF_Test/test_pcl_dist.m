folderPath = '/Users/hsimsir/Documents/CS554_Computer_Vision_Project/point_clouds';
fileExtension = '*.ply';

% Get a list of all .ply files in the folder
plyFiles = dir(fullfile(folderPath, fileExtension));

% Ensure there are at least two point cloud files
if numel(plyFiles) < 2
    error('There are fewer than two .ply files in the specified folder.');
end

% Read the first two point clouds
pcl1 = pcread(fullfile(folderPath, plyFiles(1).name));
pcl2 = pcread(fullfile(folderPath, plyFiles(2).name));

% Convert the point clouds to matrices (if needed by pcl_dist)
pcl1Matrix = pcl1.Location;
pcl2Matrix = pcl2.Location;

% Call the function pcl_dist
[pcl_SQ, SQ_pcl] = pcl_dist(pcl1Matrix, pcl2Matrix);

% Display results
fprintf('residue: \n');
disp(pcl_SQ + SQ_pcl);
% source_filepath = 'source.mat';
% target_filepath = 'target.mat';

source_folder = 'data/00000000';

source_filepath =  [source_folder, '00.pcd'];
normals_filepath = [source_folder, '00_normal.pcd'];
target_filepath =  [source_folder, '01.pcd'];

k = 1000;

%sampling method
method = 'all';

% load the point clouds and normals
[source, target, normals] = loadData(source_filepath, target_filepath, normals_filepath);

% start stopwatch
tic;

% run ICP
[R, t, iterations, RMS] = ICP(source, target, normals, k, method);

% stop stopwatch
time = toc;

% display results
disp(['method: ', method]);
disp(['RMS: ', num2str(RMS, '%.6f')]);
disp(['elapsed time: ', num2str(time, '%.2f'), ' s']);

% transform source points
source = transformPoints(source, R, t);

% plot results
plot3(source(:,1), source(:,2), source(:,3), '.', 'MarkerSize', 3, 'color', 'blue');
hold on;
plot3(target(:,1), target(:,2), target(:,3), '.', 'MarkerSize', 3);
hold off;


% source_filepath = 'source.mat';
% target_filepath = 'target.mat';

source_filepath =  'data/0000000000.pcd';
normals_filepath = 'data/0000000000_normal.pcd';
target_filepath =  'data/0000000003.pcd';

k = 1000;
method = 'normals';

[source, target, normals] = loadData(source_filepath, target_filepath, normals_filepath);

[R, t] = ICP(source, target, normals, k, method);

source = transformPoints(source, R, t);

plot3(source(:,1), source(:,2), source(:,3), '.', 'MarkerSize', 3, 'color', 'blue');
hold on;
plot3(target(:,1), target(:,2), target(:,3), '.', 'MarkerSize', 3);
hold off;


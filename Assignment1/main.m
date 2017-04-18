% source_filepath = 'source.mat';
% target_filepath = 'target.mat';

source_filepath = 'data/0000000000.pcd';
target_filepath = 'data/0000000001.pcd';

k = 1000;
method = 'normals';

ICP

plot3(source(:,1), source(:,2), source(:,3), '.', 'MarkerSize', 3);
hold on;
plot3(target(:,1), target(:,2), target(:,3), '.', 'MarkerSize', 3);
hold off;


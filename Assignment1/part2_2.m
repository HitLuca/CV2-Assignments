% cloud points folder
datafolder = 'data/00000000';

% sampling parameters
sampling_rate = 2;
method = 'all';
k = 1000;

for i=0:sampling_rate:98
    i
    if i < 10
    source_filepath = strcat(datafolder, '0', num2str(i),'.pcd');
    else
        source_filepath = strcat(datafolder, num2str(i),'.pcd');
    end
    
    % load the data
    [source, target, ~] = loadData(source_filepath, NaN, NaN);

    if i==0
         prev_cloud = source;
    else
        next_cloud = source;
        
        % match new frame to previous one
        [R, t] = ICP(next_cloud, prev_cloud, NaN, k, method);
        
        % merge the frames
        prev_cloud = (prev_cloud - t) *pinv(R'); 
        prev_cloud = [prev_cloud; next_cloud];
    end
end

% downsample
prev_cloud2 = prev_cloud(1:10:size(prev_cloud,1),:);

% plot of the point cloud
figure
plot3(prev_cloud(:,1), prev_cloud(:,2), prev_cloud(:,3),'.', 'MarkerSize', 3);
axis equal

% same cloud but with reduction in number of points for more clarity
figure
plot3(prev_cloud2(:,1), prev_cloud2(:,2), prev_cloud2(:,3),'.', 'MarkerSize', 3);
axis equal

%----- point clouds were saved and can be submitted upon request
% save Cloud_Person.mat prev_cloud;
% save Cloud_Person2.mat prev_cloud2;
% load Cloud_Person2.mat


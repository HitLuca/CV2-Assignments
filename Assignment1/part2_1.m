datafolder = 'data/00000000';

% sampling parameters
sampling_rate = 1;
method = 'all';
k = 1000;

for i=1:sampling_rate:98
    i
    % Load Point Clouds
    if source_index < 10
        source_filepath = strcat(datafolder,'0',num2str(i),'.pcd');
    else
        source_filepath = strcat(datafolder, num2str(i),'.pcd');
    end
    if target_index < 10
        target_filepath = strcat(datafolder, '0', num2str(i-1),'.pcd');
    else
        target_filepath = strcat(datafolder, num2str(i-1),'.pcd');
    end
    
    [source, target, ~] = loadData(source_filepath, target_filepath, NaN);
    
    % Merge Two Frames
    [R, t] = ICP(source, target, NaN, k, method);
    Frame_X_Y = transformPoints(source, R, t);
    
    % merge the merged frame with the last frame
    if i==1
        Frame_X_Y_Z = Frame_X_Y;
    else
        % run ICP and merge the frames
       [R, t] = ICP(Frame_X_Y, Frame_X_Y_Z, NaN, k, method);
       Frame_X_Y_Z =(Frame_X_Y_Z - t) *pinv(R'); 
    end
    
    Frame_X_Y_Z = [Frame_X_Y_Z; Frame_X_Y];
end

% downsample 
Frame_X_Y_Z_2 = Frame_X_Y_Z(1:10:size(Frame_X_Y_Z,1),:);

% plot of the point cloud but with reduction in number of points for more clarity
figure
plot3(Frame_X_Y_Z_2(:,1), Frame_X_Y_Z_2(:,2), Frame_X_Y_Z_2(:,3),'.', 'MarkerSize', 3);
axis equal


%-----
% save Frame_X_Y_ZF.mat Frame_X_Y_Z;
% save Frame_X_Y_Z_2F.mat Frame_X_Y_Z_2;

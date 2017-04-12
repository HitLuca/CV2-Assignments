source_filepath = 'data/0000000000.pcd';
target_filepath = 'data/0000000005.pcd';
k = 1000;
method = 'uniform';
iteration = 0;

[source, target] = loadData(source_filepath, target_filepath);

R = eye(3);
t = zeros(1, 3);

sampled_source = sample(source, method, k);
matched_target = match(sampled_source, target);

rms = RMS(sampled_source, matched_target);
disp(['Iteration ', num2str(iteration), ': ', num2str(rms)]);

while true
    iteration = iteration + 1;
    
    mean_source = sum(sampled_source, 1) / size(sampled_source, 1);
    mean_target = sum(matched_target, 1) / size(matched_target, 1);

    centered_source = bsxfun(@minus, sampled_source, mean_source); 
    centered_target = bsxfun(@minus, matched_target, mean_target);
    
    [R, t] = calculateRt(centered_source, centered_target, mean_source, mean_target);
    
    new_source = transform_points(source, R, t);
    
    sampled_source = sample(new_source, method, k);
    matched_target = match(sampled_source, target);
    
    new_rms = RMS(sampled_source, matched_target);
    disp(['Iteration ', num2str(iteration), ': ', num2str(new_rms)]);
    
    if new_rms >= rms
        break;
    else
        rms = new_rms;
        source = new_source;
    end
end

plot3(source(:,1), source(:,2), source(:,3), '.', 'MarkerSize', 3);
hold on;
plot3(target(:,1), target(:,2), target(:,3), '.', 'MarkerSize', 3);
hold off;

%% Support functions

function [source, target] = loadData(source_filepath, target_filepath)
    source = readPcd(source_filepath);
    target = readPcd(target_filepath);
    source = source(:, 1:3);
    target = target(:, 1:3);

%     load('source.mat', 'source');
%     load('target.mat', 'target');
%     source = source';
%     target = target';
end

function [rms] = RMS(source, target)
    rms = sqrt(sum((norm(bsxfun(@minus, source, target)))^2)/size(source, 1));
end

function [sampled_points] = sample(points, method, k)
    if strcmp(method, 'all')
        sampled_points = points;
    else
        if strcmp(method, 'random')
            indexes = randsample(size(points, 1),k);
        elseif strcmp(method, 'uniform')
            indexes = randi(size(points, 1), k, 1);
        end
        sampled_points = points(indexes, :);
    end
end

function [matched_target] = match(source, target)
    k = dsearchn(target, source);
    matched_target = target(k, :);
end

function [transformed_points] = transform_points(points, R, t)
    transformed_points = bsxfun(@plus, points * R', t);
end

function [R, t] = calculateRt(centered_source, centered_target, mean_source, mean_target)
    S = centered_source' * centered_target;

    [U,~,V] = svd(S);

    determinant = det(V * U');
    temp = [1, 0, 0; 0, 1, 0; 0, 0, determinant];

    R = V * temp * U';
    t = mean_target - mean_source * R';
end
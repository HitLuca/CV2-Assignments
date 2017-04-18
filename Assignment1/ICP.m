%#ok<*NODEF>
%#ok<*AGROW>

iteration = 0;
[source, target] = loadData(source_filepath, target_filepath);
normals = loadNormals(method, source_filepath);
    
kd_tree = KDTreeSearcher(target);

R = eye(3);
t = zeros(1, 3);

sampled_source = sample(source, method, k, normals);
matched_target = match(sampled_source, target, kd_tree);

rms = RMS(sampled_source, matched_target);
disp(['Iteration ', num2str(iteration), ': ', num2str(rms)]);

while true
    iteration = iteration + 1;
    
    mean_source = mean(sampled_source);
    mean_target = mean(matched_target);

    centered_source = bsxfun(@minus, sampled_source, mean_source); 
    centered_target = bsxfun(@minus, matched_target, mean_target);
    
    [R, t] = calculateRt(centered_source, centered_target, mean_source, mean_target);
    
    new_source = transform_points(source, R, t);
    
    sampled_source = sample(new_source, method, k, normals);
    matched_target = match(sampled_source, target, kd_tree);
    
    new_rms = RMS(sampled_source, matched_target);
    disp(['Iteration ', num2str(iteration), ': ', num2str(new_rms)]);
    
    if new_rms >= rms
        break;
    else
        rms = new_rms;
        source = new_source;
    end
end


%% Support functions

function [source, target] = loadData(source_filepath, target_filepath)
    if contains(source_filepath, '.pcd')
        source = readPcd(source_filepath);
        target = readPcd(target_filepath);
        source = source(:, 1:3);
        target = target(:, 1:3);
    elseif contains(source_filepath, '.mat')
        load('source.mat', 'source');
        load('target.mat', 'target');
        source = source';
        target = target';
    end
end

function [rms] = RMS(source, target)
    rms = sqrt(sum((norm(bsxfun(@minus, source, target)))^2)/size(source, 1));
end

function [sampled_points] = sample(points, method, k, normals)
    if strcmp(method, 'all')
        sampled_points = points;
    else
        if strcmp(method, 'random')
            indexes = randsample(size(points, 1),k);
        elseif strcmp(method, 'uniform')
            indexes = randi(size(points, 1), k, 1);
        elseif strcmp(method, 'normals')
            total_indexes = 0;
            final_indexes = [];
            while total_indexes < k
                sampled_indexes = randi(size(points, 1), k, 1);
                [knn_indexes,~] = knnsearch(points,points(sampled_indexes, :), 'k', 10);
                for i=1:size(sampled_indexes, 1)
                    knn_normals = normals(knn_indexes(i), :);
                    if variance_check(knn_normals)
                        final_indexes = [final_indexes; sampled_indexes(i)];
                        total_indexes = total_indexes + 1;
                    end
                    if total_indexes == k
                        break
                    end
                end
            end
            indexes = final_indexes;
        end
        sampled_points = points(indexes, :);
    end
end

function [matched_target] = match(source, target, kd_tree)
    k = knnsearch(kd_tree, source);
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

function normals = loadNormals(method, filepath)
    if strcmp(method, 'normals')
        [pathstr, name, ~] = fileparts(filepath);
        normals_filepath = [pathstr, '/', name, '_normal.pcd'];
        
        normals = readPcd(normals_filepath);
        normals = normals(:, 1:3);
    else
        normals = NaN;
    end
end

function ok = variance_check(neighbors)
    ok = false;
    threshold = 0.05;
    total_variance = trace(cov(neighbors));
    if total_variance > threshold
        ok = true;
    end
end
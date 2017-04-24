% main ICP routine
function [R, t, iterations, final_RMS] = ICP(source, target, normals, k, method)
    % remove unnecessary warnings
    %#ok<*NODEF>
    %#ok<*AGROW>
    
    % if using normals sampling, center the normals
    if strcmp(method, 'normals')
        normals = normals - source;
    end

    iteration = 1;
    
    % build kd tree structure on the target
    kd_tree = KDTreeSearcher(target);

    % init R and t
    R = eye(3);
    t = zeros(1, 3);
    
    % create transformation matrix
    transformation = eye(4);
    transformation(1:3, 1:3) = R;
    transformation(1:3, 4) = t';
    
    % sample the source
    sampled_source = sample(source, method, k, normals);
    
    % match target with sampled source
    matched_target = match(sampled_source, target, kd_tree);

    % calculate RMS error
    rms = RMS(sampled_source, matched_target);
    
    disp(['Iteration ', num2str(iteration), ': ', num2str(rms)]);

    while true
        iteration = iteration + 1;

        % center source and target
        mean_source = mean(sampled_source);
        mean_target = mean(matched_target);
        centered_source = bsxfun(@minus, sampled_source, mean_source); 
        centered_target = bsxfun(@minus, matched_target, mean_target);

        % calculate R and t
        [R, t] = calculateRt(centered_source, centered_target, mean_source, mean_target);
        
        % transform source points
        new_source = transformPoints(source, R, t);

        % sample the source points
        sampled_source = sample(new_source, method, k, normals);
        
        % match target with sampled source
        matched_target = match(sampled_source, target, kd_tree);

        % determine new RMS
        new_rms = RMS(sampled_source, matched_target);
        
        disp(['Iteration ', num2str(iteration), ': ', num2str(new_rms)]);

        % if worse results break
        if new_rms >= rms
            break;
        
        % otherwise update the transformation values and continue
        else
            rms = new_rms;
            source = new_source;
            
            % calculate the new transformation
            new_transformation = eye(4);
            new_transformation(1:3, 1:3) = R;
            new_transformation(1:3, 4) = t';
            transformation = transformation * new_transformation;
        end
    end
    % extract the final R and t
    R = transformation(1:3, 1:3);
    t = transformation(1:3, 4)';
    
    iterations = iteration - 1;
    final_RMS = rms;
end

%% Support functions

% calculate the root mean squared error
function [rms] = RMS(source, target)
    rms = sqrt(sum((norm(bsxfun(@minus, source, target)))^2)/size(source, 1));
end

% sample the points with the given method
function [sampled_points] = sample(points, method, k, normals)
    % no sampling
    if strcmp(method, 'all')
        sampled_points = points;
    else
        % random sampling
        if strcmp(method, 'random')
            indexes = randsample(size(points, 1),k);
            
        % uniform sampling
        elseif strcmp(method, 'uniform')
            indexes = randi(size(points, 1), k, 1);
            
        % normals sampling
        elseif strcmp(method, 'normals')
            % knn neighbors
            nieghbors_number = 10;
            
            total_indexes = 0;
            final_indexes = [];
            
            % sample until the candidates are == k
            while total_indexes < k
                % uniformly sample k candidates
                sampled_indexes = randi(size(points, 1), k, 1);
                
                % find the neighbors_number neighbors for each candidate
                [knn_indexes,~] = knnsearch(points,points(sampled_indexes, :), 'k', nieghbors_number);
                
                % check every sampled point using normals
                for i=1:size(sampled_indexes, 1)
                    % extract the normals
                    knn_normals = normals(knn_indexes(i, :), :);
                    
                    % check the extracted normals variance
                    if variance_check(knn_normals, nieghbors_number)
                        % add the candidates
                        final_indexes = [final_indexes; sampled_indexes(i)];
                        total_indexes = total_indexes + 1;
                    end
                    
                    % break if enough candidates have been found
                    if total_indexes == k
                        break
                    end
                end
            end
            indexes = final_indexes;
        end
        % return the candidates
        sampled_points = points(indexes, :);
        
        % print of the sampled points at a given iteration, uncheck and 
        % stop at the first iteration to see the reults
        % plot3(sampled_points(:, 1), sampled_points(:, 2), sampled_points(:, 3), '.', 'MarkerSize', 5, 'color', 'red');
        % hold on;
    end
end

% match the source with the closest target
function [matched_target] = match(source, target, kd_tree)
    k = knnsearch(kd_tree, source);
    matched_target = target(k, :);
end

% calculate optimal R and t
function [R, t] = calculateRt(centered_source, centered_target, mean_source, mean_target)
    % covariance matrix
    S = centered_source' * centered_target;
    
    % SVD of S
    [U,~,V] = svd(S);

    determinant = det(V * U');
    temp = [1, 0, 0; 0, 1, 0; 0, 0, determinant];

    % calculate optimal R and t
    R = V * temp * U';
    t = mean_target - mean_source * R';
end

% check the variance of the normals
function ok = variance_check(neighbors, nieghbors_number)
    ok = false;
    
    % user-defined threshold
    threshold = 5e-5;
    
    % total variance calculation, normalized by neighbors_number
    total_variance = trace(cov(neighbors)) / nieghbors_number;
    
    % if the variance is above the threshold, return true
    if total_variance > threshold
        ok = true;
    end
end
% RANSAC algorithm
function [final_F, final_inliers] = RANSAC(p1, p2, threshold)    
    %#ok<*AGROW>  
     
    % maximum number of tries for finding a good F
    max_tries = 50;
    
    % final parameters
    max_inliers = 0;
    final_F = 0;
    final_inliers = [];
    
    % normalization of p1 and p2
    [p1_norm, T1] = normalize_points(p1);
    [p2_norm, T2] = normalize_points(p2);
    
    % loop counter
    tries = 0;
    
    % infinite loop with automatic stop
    while true
        inliers = 0;
        temp_inliers = [];
        
        % random sampling of 8 points
        samples = randsample(size(p1_norm, 2), 8); 
        sampled_p1 = p1_norm(:, samples);
        sampled_p2 = p2_norm(:, samples);
        
        % find the F matrix
        F = normalized_eight_points(sampled_p1, sampled_p2, T1, T2);
        
        % calculate the sampson distance of the points
        d = sampson_distance(p1_norm, p2_norm, F);
        
        % for every point
        for i=1:size(d)
            % if the distance is less than the threshold
            if d(i) < threshold
                % add 1 to the inliers counter
                inliers = inliers + 1;
                % add the point index to the inliers
                temp_inliers = [temp_inliers; i];
            end
        end
        
        % if the inliers are more than the past 
        if inliers > max_inliers
            % update the parameters
            max_inliers = inliers;
            final_F = F;
            final_inliers = temp_inliers;
            % reset the loop counter
            tries = 0;
            
        % otherwise increase the loop counter
        else
            tries = tries + 1;
        end
        
        % if the tries have reached the maximum exit the loop
        if tries == max_tries
            break
        end
    end
end

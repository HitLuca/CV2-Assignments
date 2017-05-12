function [final_F, final_inliers] = RANSAC(p1, p2, threshold)    
    %#ok<*AGROW>    
    max_inliers = 0;
    final_F = 0;
    
    final_inliers = [];
    
    [p1_norm, T1] = normalize_points(p1);
    [p2_norm, T2] = normalize_points(p2);
    
    improvement = true;
    tries = 0;
    
    while improvement
        inliers = 0;
        temp_inliers = [];
        
        samples = randsample(size(p1_norm, 2), 8); 

        sampled_p1 = p1_norm(:, samples);
        sampled_p2 = p2_norm(:, samples);
        
        F = normalized_eight_points(sampled_p1, sampled_p2, T1, T2);
        
        d = sampson_distance(p1_norm, p2_norm, F);
        for i=1:size(d)
            if d(i) < threshold
                inliers = inliers + 1;
                temp_inliers = [temp_inliers; i];
            end
        end
        if inliers > max_inliers
            max_inliers = inliers;
            final_F = F;
            final_inliers = temp_inliers;
            tries = 0;
        else
            tries = tries + 1;
        end
        if tries == 10
            improvement = false;
        end
    end
end

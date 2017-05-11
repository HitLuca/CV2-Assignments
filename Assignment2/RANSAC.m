function [final_F, final_inliers] = RANSAC(p1, p2, T1, T2, iterations, threshold)    
    %#ok<*AGROW>    
    max_inliers = 0;
    final_F = 0;
    
    final_inliers = [];
    
    for k=1:iterations
        temp_inliers = [];
        samples = randsample(size(p1, 2), 8); 

        sampled_p1 = p1(:, samples);
        sampled_p2 = p2(:, samples);
        
        F = normalized_eight_points(sampled_p1, sampled_p2, T1, T2);

        inliers = 0;
        for i=1:size(p1, 2)
            d = sampson_distance(p1(:, i), p2(:, i), F);
            if d < threshold
                inliers = inliers + 1;
                temp_inliers = [temp_inliers; i];
            end
        end
        if inliers > max_inliers
            max_inliers = inliers;
            final_F = F;
            final_inliers = temp_inliers;
        end
    end
end

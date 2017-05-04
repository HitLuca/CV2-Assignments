function [final_F] = RANSAC(p1, p2, T1, T2, iterations, threshold)    
    max_inliers = 0;
    final_F = 0;
    
    for k=1:iterations
        samples = randsample(size(p1, 2), 8); 

        sampled_p1 = p1(:, samples);
        sampled_p2 = p2(:, samples);
        
        F = normalized_eight_points(sampled_p1, sampled_p2, T1, T2);

        inliers = 0;
        for i=1:size(p1, 2)
            d = sampson_distance(p1(:, i), p2(:, i), F);
            if d < threshold
                inliers = inliers + 1;
            end
        end
        if inliers > max_inliers
            max_inliers = inliers;
            final_F = F;
        end
    end
end

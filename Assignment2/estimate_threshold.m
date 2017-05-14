% extra. Automatic RANSAC threshold estimation using sampson distances
function [ threshold ] = estimate_threshold(p1, p2, F)
    % calculate the total sampson distance when using the given F
    total_distance = 0;
    for i=1:size(p1, 2)
        total_distance = total_distance + sampson_distance(p1(:, i), p2(:, i), F);
    end
    
    % average it
    total_distance = total_distance / size(p1, 2);
    
    % calculate the RANSAC threshold
    threshold = total_distance / 1000;
end


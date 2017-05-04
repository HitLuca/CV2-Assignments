function [ threshold ] = estimate_threshold(p1, p2, F)
    total_distance = 0;
    for i=1:size(p1, 2)
        total_distance = total_distance + sampson_distance(p1(:, i), p2(:, i), F);
    end
    total_distance = total_distance / size(p1, 2);
    threshold = total_distance / 1000;
end


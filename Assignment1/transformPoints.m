function [transformed_points] = transformPoints(points, R, t)
    transformed_points = bsxfun(@plus, points * R', t);
end
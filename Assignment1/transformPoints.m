% transform the points with the given rotation and translation
function [transformed_points] = transformPoints(points, R, t)
    transformed_points = bsxfun(@plus, points * R', t);
end
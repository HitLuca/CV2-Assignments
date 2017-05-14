% normalize the input points
function [normalized_p, T] = normalize_points(p)
    % calculate mean of the x and y coords
    x_mean = mean(p(1, :));
    y_mean = mean(p(2, :));
    
    % calculate d
    d = sum(sqrt((p(1, :) - x_mean).^2 + (p(2, :) - y_mean).^2)) / size(p, 2);
    
    % create T
    T = [sqrt(2)/d, 0, -x_mean * sqrt(2)/d;
        0, sqrt(2)/d, -y_mean*sqrt(2)/d;
        0, 0, 1];
    
    % normalize p
    normalized_p = T*p;
end


function [normalized_p, T] = normalize_points(p)
    x_mean = mean(p(1, :));
    y_mean = mean(p(2, :));
    d = sum(sqrt((p(1, :) - x_mean).^2 + (p(2, :) - y_mean).^2)) / size(p, 2);
    
    T = [sqrt(2)/d, 0, -x_mean * sqrt(2)/d;
        0, sqrt(2)/d, -y_mean*sqrt(2)/d;
        0, 0, 1];
    normalized_p = T*p;
end


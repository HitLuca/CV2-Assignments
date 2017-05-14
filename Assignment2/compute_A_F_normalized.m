% calculate the A and F matrices from points p1 and p2, normalized version
function [A, F] = compute_A_F_normalized(p1, p2)
    % empty A matrix
    A = ones(size(p1, 2), 9);
    
    A(:, 1) = (p1(1, :) .* p2(1, :))'; % x1x2
    A(:, 2) = (p1(1, :) .* p2(2, :))'; % x1y2
    A(:, 3) = p1(1, :)'; % x1
    A(:, 4) = (p1(2, :) .* p2(1, :))'; % y2
    A(:, 5) = (p1(2, :) .* p2(2, :))'; % y1y2
    A(:, 6) = p1(2, :)'; % y1
    A(:, 7) = p2(1, :)'; % x2
    A(:, 8) = p2(2, :)'; % y2
    
    % svd decomposition
    [~,~,V] = svd(A, 'econ');
    
    % F calculation
    F = V(:, end);
    
    % reshape
    F = [F(1:3), F(4:6), F(7:9)];
end
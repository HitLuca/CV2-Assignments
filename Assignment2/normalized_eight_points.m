% normalized eight points algorithm
function [F] = normalized_eight_points(p1, p2, T1, T2)
    % compute the matrix F
    [~, F] = compute_A_F_normalized(p1, p2);
    
    % correct F
    F = correct_F(F);
    
    % recompute F using the transformation matrices
    F = T2' * F * T1;
end
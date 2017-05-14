% eight points algorithm
function [F] = eight_points(p1, p2)
    % compute the matrix F
    [~, F] = compute_A_F(p1, p2);
    
    % correct F
    F = correct_F(F);
end
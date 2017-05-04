function [F] = eight_points(p1, p2)
    [A, F] = compute_A_F(p1, p2);
    F = correct_F(F);
end
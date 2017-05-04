function [F] = normalized_eight_points(p1, p2, T1, T2)
    [A, F] = compute_A_F_normalized(p1, p2);
    F = correct_F(F);
    F = T2' * F * T1;
end
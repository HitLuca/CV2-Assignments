function [A, F] = compute_A_F(p1, p2)
    A = ones(size(p1, 2), 9);
    A(:, 1) = (p1(1, :) .* p2(1, :))';
    A(:, 2) = (p1(1, :) .* p2(2, :))';
    A(:, 3) = p1(1, :)';
    A(:, 4) = (p1(2, :) .* p2(1, :))';
    A(:, 5) = (p1(2, :) .* p2(2, :))';
    A(:, 6) = p1(2, :)';
    A(:, 7) = p2(1, :)';
    A(:, 8) = p2(2, :)';
    
    [U,S,V] = svd(A, 'econ');

    [~, index] = min(diag(S));

    F = V(:, index);
    F = [F(1:3), F(4:6), F(7:9)];
end
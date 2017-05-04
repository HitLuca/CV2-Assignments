function [F] = correct_F(F)
    [U,S,V] = svd(F, 'econ');
    x = diag(S);
    [~, index] = min(x);
    x(index) = 0;
    S = diag(x);
    F = U * S * V'; 
end
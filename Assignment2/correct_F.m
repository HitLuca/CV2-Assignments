% correct the F matrix in the eight points algortihm
function [F] = correct_F(F)
    % svd decomposition of F
    [U,S,V] = svd(F, 'econ');
    
    x = diag(S);
    
    % index of the smallest singular value
    [~, index] = min(x);
    
    % index set to 0
    x(index) = 0;
    
    % recomputation of F
    S = diag(x);
    F = U * S * V'; 
end
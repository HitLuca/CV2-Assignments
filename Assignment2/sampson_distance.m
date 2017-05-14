% sampson distance calculation using matrix implementation
function [d] = sampson_distance(p1, p2, F)
    Fp1 = (F*p1).^2;
    Fp2 = (F'*p2).^2;
    
    d = diag((p2'*F*p1).^2 ./ (Fp1(1, :) + Fp1(2, :) + Fp2(1, :) + Fp2(2, :)));
end

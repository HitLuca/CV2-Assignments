[source, target] = loadData();

mean_target = sum(target, 1) / size(target, 1);
centered_target = bsxfun(@minus, target, mean_target);

R = eye(3);
t = zeros(1, 3);

plot3(source(:,1), source(:,2), source(:,3));
hold on;
plot3(target(:,1), target(:,2), target(:,3));
hold off;

[rms, source] = RMS(source, R, t, target);
rms


while true
    mean_source = sum(source, 1) / size(source, 1);

    centered_source = bsxfun(@minus, source, mean_source); 

    S = centered_source' * centered_target;

    [U,Sigma,V] = svd(S);

    determinant = det(V * U');
    temp = [1, 0, 0; 0, 1, 0; 0, 0, determinant];

    R = V * temp * U';
    t = mean_target - mean_source * R';
    
    [new_rms, new_source] = RMS(source, R, t, target);
    new_rms
    
    if new_rms >= rms
        break;
    else
        rms = new_rms;
        source = new_source;
    end
end



%% Support functions

function [source, target] = loadData()
    load('source.mat', 'source');
    load('target.mat', 'target');
    source = source';
    target = target';
end

function [rms, new_source] = RMS(source, R, t, target)
    new_source = bsxfun(@plus, source * R', t);

    k = dsearchn(target, source);
    
    rms = sqrt(sum((norm(bsxfun(@minus, new_source, target(k))))^2)/size(new_source, 1));
end
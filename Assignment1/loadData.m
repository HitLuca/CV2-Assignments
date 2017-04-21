function [source, target] = loadData(source_filepath, target_filepath)
    if contains(source_filepath, '.pcd')
        source = readPcd(source_filepath);
        target = readPcd(target_filepath);
        
        condition=source(:,3)>2;
        source(condition,:)=[];
        source = source(:, 1:3);
        
        condition=target(:,3)>2;
        target(condition,:)=[];
        target = target(:, 1:3);
    elseif contains(source_filepath, '.mat')
        load('source.mat', 'source');
        load('target.mat', 'target');
        source = source';
        target = target';
    end
end

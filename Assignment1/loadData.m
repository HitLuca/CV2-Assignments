function [source, target, normals] = loadData(source_filepath, target_filepath, normals_filepath)
    if contains(source_filepath, '.pcd')
        source = readPcd(source_filepath);
        target = readPcd(target_filepath);
        source_condition=source(:,3)>2;
        source(source_condition,:)=[];
        source = source(:, 1:3);
        
        target_condition=target(:,3)>2;
        target(target_condition,:)=[];
        target = target(:, 1:3);
        
        if contains(normals_filepath, 'normal')
            normals = readPcd(normals_filepath);
            normals(source_condition,:)=[];
            normals = normals(:, 1:3);
        else
            normals = NaN;
        end
    elseif contains(source_filepath, '.mat')
        load('source.mat', 'source');
        load('target.mat', 'target');
        source = source';
        target = target';
    end
end

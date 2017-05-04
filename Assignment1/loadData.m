% loading of the cloud of points and normals
function [source, target, normals] = loadData(source_filepath, target_filepath, normals_filepath)
    if contains(source_filepath, '.pcd')
        source = readPcd(source_filepath);
        
        %remove the background points
        source_condition=source(:,3)>2;
        source(source_condition,:)=[];
        
        % remove unnecessary data
        source = source(:, 1:3);
        
        if ~isnan(target_filepath)
            target = readPcd(target_filepath);
            
            %remove the background points
            target_condition=target(:,3)>2;
            target(target_condition,:)=[];
            
            % remove unnecessary data
            target = target(:, 1:3);
        else
            target = NaN;
        end
        
        if ~isnan(normals_filepath)
            normals = readPcd(normals_filepath);
            
            %remove the background points
            normals(source_condition,:)=[];
            
            % remove unnecessary data
            normals = normals(:, 1:3);
        else
            normals = NaN;
        end
    elseif contains(source_filepath, '.mat')
        load(source_filepath, 'source');
        load(target_filepath, 'target');
        
        % transpose the datasets
        source = source';
        target = target';
        
        % no normals have been associated with the examples
        normals = NaN;
    end
end

% image chaining

clear;
source_path = 'House/frame000000';
threshold = 0.5;

% list containing for each cell all the coordinates of a point (from
% different cameras)
matched_p_coords = {};

% point view matrix
point_view_matrix = [];

% load the first image
image_prec = load_image(source_path, 1);

% extract sift descriptors
[f_prec, d_prec] = vl_sift(image_prec);

[h, w] = size(image_prec);

%save initial descriptors for 49-1 comparison
f_1 = f_prec;
d_1 = d_prec;

% for each subsequent image
for im_index = 2:49
    disp(['image ', num2str(im_index)]);
    
    % update the point view matrix with the new matches
    [point_view_matrix, matched_p_coords, f_next, d_next] = update_pwm(source_path, point_view_matrix, matched_p_coords, threshold, im_index, im_index-1, f_prec, d_prec);
    
    % save the descriptors for next comparison
    f_prec = f_next;
    d_prec = d_next;
end
    
% final 49-1 comparison
disp(['image ', num2str(1)]);
[point_view_matrix, matched_p_coords, ~, ~] = update_pwm(source_path, point_view_matrix, matched_p_coords, threshold, 49, 1, f_1, d_1);
    
% show the binary point view matrix
figure()
imshow(point_view_matrix, [])

% create the final point view matrix with coordinates
pwm = create_pwm(matched_p_coords);

% save the matrix
dlmwrite('myPwm.txt', pwm,'delimiter',' ');


%%

% update the input matrix with the new matches emerging from prev_index and
% curr_index images
function [point_view_matrix, matched_p_coords, f_next, d_next] = update_pwm(source_path, point_view_matrix, matched_p_coords, threshold, curr_index, prev_index, f_prec, d_prec)
    % load the next image
    image_next = load_image(source_path, curr_index);
    
    % extract sift descriptors
    [f_next, d_next] = vl_sift(image_next);
    
    % match the descriptors of the two images
    [p_prec, p_next] = match_images(f_prec, d_prec, f_next, d_next);
    
    % run the RANSAC algorithm to find the indexes of the inliers
    [~, matched_p_indexes] = RANSAC(p_prec, p_next, threshold);
    
    % for each pair of inliers
    for i=1:size(matched_p_indexes)
        m_index = matched_p_indexes(i);
        
        % check if the pair is already saved in matched_p_coords and add if
        % not
        [index, matched_p_coords] = check_for_point(matched_p_coords, p_prec(:, m_index), p_next(:, m_index), curr_index);
        
        % update the values of the pwm
        point_view_matrix(prev_index, index) = true;
        point_view_matrix(curr_index, index) = true;
    end
end

% check if a pair of points has already been saved in the matches list
function [index, matches] = check_for_point(matches, point1, point2, im_index)
    p1_found = false;
    p2_found = false;
    
    % for each cell in the list
    for i=1:size(matches, 2)
        m = matches{i};
        
        % for each column of the matrix
        for column=1:size(m, 2)
            
            % multiple checks in order to avoid 3 matches for every point
            % (x-y-z). Great speed up
            
            % if the x coordinate matches the first element
            if m(1, column) == point2(1)
                % and the y coordinate matches the second element
                if m(1:3, column) == point2
                    % p2 is already saved
                    p2_found = true;
                end
            end
            % if the x coordinate matches the first element
            if m(1, column) == point1(1)
                % and the y coordinate matches the second element
                if m(1:3, column) == point1
                    % p1 is already saved
                    p1_found = true;
                    % save the index of the cell
                    index = i;
                    break
                end
            end

        end
    end
    
    % if p1 is saved and p2 has not been found
    if p1_found && ~ p2_found
        % add p2 to the matrix
        matches{index} = [matches{index}, [point2;im_index]];
        
    % otherwise (p1 and p2 not found)
    else
        % add both to the matrix, along with the image index from where
        % they are from
        matches{end+1} = [[point1;im_index-1], [point2;im_index]];
        % update the index
        index = size(matches, 2);
    end
end

% create the final point view matrix from the list of matches
function [pwm] = create_pwm(matched_p_coords)
    m = 49;
    n = size(matched_p_coords, 2);
    
    % empty pwm
    pwm = zeros(2*m, n);
    
    % for every cell
    for k=1:size(matched_p_coords, 2)
        % for every column of the matrix
        for column=1:size(matched_p_coords{k}, 2)
            % extract the element
            element = matched_p_coords{k}(:, column);
            % update the pwm using the last value of element (image index)
            pwm((element(4)-1)*2+1, k) = element(1);
            pwm((element(4)-1)*2+2, k) = element(2);
        end
    end
end
clear;
source_path = 'House/frame000000';
n_matches = 100;
iterations = 100; %1000;


matched_p_coords = {};
point_view_matrix = [];
next_cell = 1;

for im_index = 1:30
    im_index
    if im_index < 9    
        image1 = single(imread([source_path, '0', num2str(im_index), '.png']));
        image2 = single(imread([source_path, '0', num2str(im_index+1), '.png']));
    elseif im_index == 9
        image1 = single(imread([source_path, '0', num2str(im_index), '.png']));
        image2 = single(imread([source_path, num2str(im_index+1), '.png']));
    else
        image1 = single(imread([source_path, num2str(im_index), '.png']));
        image2 = single(imread([source_path, num2str(im_index+1), '.png']));
    end
    [h, w] = size(image1);

    [p1, p2] = match_images(image1, image2, n_matches);

    matched_p_indexes = match_points(p1, p2, iterations);

    for i=1:size(matched_p_indexes)
        m_index = matched_p_indexes(i);
        [found, index] = check_for_point(matched_p_coords, p1(:, m_index));
        if found
            point_view_matrix(im_index, index) = true;
            point_view_matrix(im_index+1, index) = true;
        else
            point_view_matrix(im_index, next_cell) = true;
            point_view_matrix(im_index+1, next_cell) = true;

            matched_p_coords{next_cell} = [p1(:, m_index), p2(:, m_index)];
            next_cell = next_cell + 1;
        end
    end
end

imshow(point_view_matrix)
% imshow([image1, image2], []);
% hold on
% scatter(p1(1, matched_p_indexes), p1(2, matched_p_indexes));
% scatter(p2(1, matched_p_indexes) + w, p2(2, matched_p_indexes));


%%
function [matched_p_indexes] = match_points(p1, p2, iterations)
    [p1_norm, T1] = normalize_points(p1);
    [p2_norm, T2] = normalize_points(p2);

    F = normalized_eight_points(p1_norm, p2_norm, T1, T2);
    threshold = estimate_threshold(p1_norm, p2_norm, F);

    disp(['threshold: ', num2str(threshold)]);

    [F, matched_p_indexes] = RANSAC(p1_norm, p2_norm, T1, T2, iterations, threshold);

    disp(['average: ' num2str(mean(mean(p2_norm'*F*p1_norm)))])
end

function [found,index] = check_for_point(matches, point)
    found = false;
    index = -1;
    
    for i=1:size(matches, 2)
        m = matches{i};
        for column=1:size(m, 2)
            if m(column) == point
                found = true;
                index = i;
                break;
            end
        end
    end
end
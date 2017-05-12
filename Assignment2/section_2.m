clear;
source_path = 'House/frame000000';
threshold = 1;

matched_p_coords = {};
point_view_matrix = [];

image_prec = load_image(source_path, 1);
[f_prec, d_prec] = vl_sift(image_prec);
[h, w] = size(image_prec);

for im_index = 2:49 %TODO: change to 49
    disp(['image ', num2str(im_index)]);
    image_next = load_image(source_path, im_index);
    [f_next, d_next] = vl_sift(image_next);
    
    [p_prec, p_next] = match_images(f_prec, d_prec, f_next, d_next);
    matched_p_indexes = match_points(p_prec, p_next, threshold);

    for i=1:size(matched_p_indexes)
        m_index = matched_p_indexes(i);
        [index, matched_p_coords] = check_for_point(matched_p_coords, p_prec(:, m_index), p_next(:, m_index), im_index);
        point_view_matrix(im_index-1, index) = true;
        point_view_matrix(im_index, index) = true;
    end
    
    image_prec = image_next;
    f_prec = f_next;
    d_prec = d_next;
end
    
figure()
imshow(point_view_matrix, [])

pwm = create_pwm(matched_p_coords);
% TODO: add comparison between first and last image

figure()
imshow(pwm, []);


%%
function [matched_p_indexes] = match_points(p1, p2, threshold)    
    [~, matched_p_indexes] = RANSAC(p1, p2, threshold);
end

function [index, matches] = check_for_point(matches, point1, point2, im_index)
    p1_found = false;
    p2_found = false;
    
    for i=1:size(matches, 2)
        m = matches{i};
        for column=1:size(m, 2)
            if m(1, column) == point2(1)
                if m(1:3, column) == point2
                    p2_found = true;
                end
            end
            if m(1, column) == point1(1)
                if m(1:3, column) == point1
                    p1_found = true;
                    index = i;
                    break
                end
            end

        end
    end
    
    if p1_found && ~ p2_found
        matches{index} = [matches{index}, [point2;im_index]];
    else
        matches{end+1} = [[point1;im_index-1], [point2;im_index]];
        index = size(matches, 2);
    end
end

function [p1, p2] = match_images(f1, d1, f2, d2)
    matches = vl_ubcmatch(d1, d2, 2);

    p1 = [f1(1:2, matches(1,:)); ones(1, size(matches, 2))];
    p2 = [f2(1:2, matches(2,:)); ones(1, size(matches, 2))];
end

function [pwm] = create_pwm(matched_p_coords)
    m = 49;
    n = size(matched_p_coords, 2);
    pwm = zeros(2*m, n);
    for k=1:size(matched_p_coords, 2)
        for column=1:size(matched_p_coords{k}, 2)
            element = matched_p_coords{k}(:, column);
            pwm((element(4)-1)*2+1, k) = element(1);
            pwm((element(4)-1)*2+2, k) = element(2);
        end
    end
end

function [image] = load_image(source_path, im_index)
    if im_index < 10    
        image = single(imread([source_path, '0', num2str(im_index), '.png']));
    else
        image = single(imread([source_path, num2str(im_index), '.png']));
    end
end
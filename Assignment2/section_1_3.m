% normalized eight point algorithm with RANSAC

clear;
source_path = 'House/frame000000';
threshold = 0.01; % RANSAC threshold

image1 = load_image(source_path, 1);
image2 = load_image(source_path, 3);

[h, w] = size(image1);

% extract sift descriptors
[f1, d1] = vl_sift(image1);
[f2, d2] = vl_sift(image2);

% create p1-p2 matchings
[p1, p2] = match_images(f1, d1, f2, d2);

% calculate the F matrix along with the matching inliers
[F, matched_p_indexes] = RANSAC(p1, p2, threshold);

% normalize the matchings
[p1_norm, ~] = normalize_points(p1);
[p2_norm, ~] = normalize_points(p2);

% average value for p2_norm^T*F*p1_norm
disp(['average: ' num2str(mean(mean(p2_norm'*F*p1_norm)))])

% plot the epipolar lines
plot_epipolar_lines(p1, p2, F, matched_p_indexes, image1, image2);


% epipolar lines plotting
function [] = plot_epipolar_lines(p1, p2, F, matched_p_indexes, image1, image2)
    [~, w] = size(image1);

    % left image points
    L_points = [p1(1, matched_p_indexes)', p1(2, matched_p_indexes)'];
    
    % right image points
    R_points = [p2(1, matched_p_indexes)', p2(2, matched_p_indexes)'];

    % svd decomposition of F
    [U, S, V]= svd (F, 'econ');
    
    % rightmost singular value set to 0
    S(3,3) = 0;

    % recomputation of F
    FF = U*S*V';

    % calculate left-image epipolars
    epipole = V(:,3)/V(3,3);

    epi_line_x_left = 1:size(image1,2);
    epi_line_y_left = L_points(:,2) + (epi_line_x_left-L_points(:,1)).*(epipole(2)-L_points(:,2))./(epipole(1)-L_points(:,1));

    % calculate corresponding right-image epipolars
    rp = FF*[L_points,ones(size(L_points,1),1)]';
    rp = rp';
    epi_line_x_right=1:size(image2,2);
    epi_line_y_right=(-rp(:,3)-rp(:,1)*epi_line_x_right)./rp(:,2);

    % plot the epipolar lines for both images next to each other
    figure
    imshow([image1,image2],[]); 
    hold on
    for i=1:50
        plot(L_points(i,1),L_points(i,2),'r*')
        plot(epi_line_x_left,epi_line_y_left(i,:))   

        plot(R_points(i,1)+w,R_points(i,2),'b*')
        plot(epi_line_x_right+w,epi_line_y_right(i,:))   
    end
    title('corresponding epipolar lines for the two images');
end

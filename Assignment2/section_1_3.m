clear;
source_path = 'House/frame000000';
n_matches = 50;
iterations = 1000;
threshold = 0.01;

image1 = single(imread([source_path, '01.png']));
image2 = single(imread([source_path, '02.png']));

[h, w] = size(image1);

[p1, p2] = match_images(image1, image2, n_matches);

[p1_norm, ~] = normalize_points(p1);
[p2_norm, ~] = normalize_points(p2);
    
[F, matched_p_indexes] = RANSAC(p1, p2, iterations, threshold);

disp(['average: ' num2str(mean(mean(p2_norm'*F*p1_norm)))])

imshow([image1, image2], []);
hold on
scatter(p1(1, matched_p_indexes), p1(2, matched_p_indexes));
scatter(p2(1, matched_p_indexes) + w, p2(2, matched_p_indexes));
clear;
source_path = 'House/frame000000';
n_matches = 50;
iterations = 1000;

image1 = single(imread([source_path, '01.png']));
image2 = single(imread([source_path, '02.png']));

[h, w] = size(image1);

[p1, p2] = match_images(image1, image2, n_matches);

[p1_norm, T1] = normalize_points(p1);
[p2_norm, T2] = normalize_points(p2);
    
F = normalized_eight_points(p1_norm, p2_norm, T1, T2);
threshold = estimate_threshold(p1_norm, p2_norm, F);

disp(['threshold: ', num2str(threshold)]);

[F, matched_p_indexes] = RANSAC(p1_norm, p2_norm, T1, T2, iterations, threshold);

disp(['average: ' num2str(mean(mean(p2_norm'*F*p1_norm)))])

imshow([image1, image2], []);
hold on
scatter(p1(1, matched_p_indexes), p1(2, matched_p_indexes));
scatter(p2(1, matched_p_indexes) + w, p2(2, matched_p_indexes));
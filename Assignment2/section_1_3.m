clear;
source_path = 'House/frame000000';
n_matches = 50;
iterations = 1000;

image1 = single(imread([source_path, '01.png']));
image2 = single(imread([source_path, '02.png']));

[h, w] = size(image1);

[p1, p2] = match_images(image1, image2, n_matches);

[p1, T1] = normalize_points(p1);
[p2, T2] = normalize_points(p2);
    
F = normalized_eight_points(p1, p2, T1, T2);
threshold = estimate_threshold(p1, p2, F);

disp(['threshold: ', num2str(threshold)]);

F = RANSAC(p1, p2, T1, T2, iterations, threshold);

disp(['average: ' num2str(mean(mean(p2'*F*p1)))])

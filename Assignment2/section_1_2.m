clear;
source_path = 'House/frame000000';
n_matches = 50;

image1 = single(imread([source_path, '01.png']));
image2 = single(imread([source_path, '02.png']));

[h, w] = size(image1);

[p1, p2] = match_images(image1, image2, n_matches);

[p1_norm, T1] = normalize_points(p1);
[p2_norm, T2] = normalize_points(p2);

F = normalized_eight_points(p1_norm, p2_norm, T1, T2);

disp(['average: ' num2str(mean(mean(p2_norm'*F*p1_norm)))])
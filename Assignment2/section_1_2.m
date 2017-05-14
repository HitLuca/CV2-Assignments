% normalized eight point algorithm

clear;
source_path = 'House/frame000000';

image1 = load_image(source_path, 1);
image2 = load_image(source_path, 3);

[h, w] = size(image1);

% extract sift descriptors
[f1, d1] = vl_sift(image1);
[f2, d2] = vl_sift(image2);

% create p1-p2 matchings
[p1, p2] = match_images(f1, d1, f2, d2);

% normalize the matchings
[p1_norm, T1] = normalize_points(p1);
[p2_norm, T2] = normalize_points(p2);

% calculate the F matrix
F = normalized_eight_points(p1_norm, p2_norm, T1, T2);

% average value for p2_norm^T*F*p1_norm
disp(['average: ' num2str(mean(mean(p2_norm'*F*p1_norm)))])
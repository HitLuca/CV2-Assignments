% eight point algorithm

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

% calculate the F matrix
F = eight_points(p1, p2);

% average value for p2^T*F*p1
disp(['average: ' num2str(mean(mean(p2'*F*p1)))])

% showing 50 matches
imshow([image1, image2], []);
hold on
scatter(p1(1, 1:50), p1(2, 1:50));
scatter(p2(1, 1:50) + w, p2(2, 1:50));
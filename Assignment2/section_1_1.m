clear;
source_path = 'House/frame000000';
n_matches = 50;

image1 = single(imread([source_path, '01.png']));
image2 = single(imread([source_path, '02.png']));

[h, w] = size(image1);

[p1, p2] = match_images(image1, image2, n_matches);

F = eight_points(p1, p2);

disp(['average: ' num2str(mean(mean(p2'*F*p1)))])

% imshow([image1, image2], []);
% hold on
% scatter(p1(1, :), p1(2, :));
% scatter(p2(1, :) + w, p2(2, :));
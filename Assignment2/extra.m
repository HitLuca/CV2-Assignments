% not required, analysis of the movements of the points contained in all views
clear;
%#ok<*AGROW>
D = importdata('myPwm.txt');
source_path = 'House/frame000000';

% take only the points contained in all views
indexes = [];
for column = 1:size(D, 2)
    if min(D(:, column)) > 0
        indexes = [indexes, column];
    end
end

D = D(:, indexes);

% extract the points coordinates
xs = [];
ys = [];
for row = 1:2:size(D,1)
    for column=1:size(D, 2)
        xs = [xs, D(row, column)]; 
        ys = [ys, D(row+1, column)];
    end
end

image1 = load_image(source_path, 1);
image2 = load_image(source_path, 25);

% two images overlay
figure()
imshow((image1 + image2)./2, []);
hold on;
scatter(xs, ys, '.')

% single image overlay
figure()
imshow(image1, []);
hold on;
scatter(xs, ys, '.')
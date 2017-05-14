% structure from motion

clear;

% import the pwm matrix
% D = importdata('PointViewMatrix.txt');
D = importdata('myPwm.txt');

% extract the indexes of the points in all views
indexes = [];
for column = 1:size(D, 2)
    if min(D(:, column)) > 0
        indexes = [indexes, column];
    end
end

% reduce D to a dense matrix of points
D = D(:, indexes);

% translate each row to the mean of the points in each row
for row = 1:size(D, 1)
    D(row, :) = D(row, :) - mean(D(row, :));
end

% svd decomposition of D
[U, W, V] = svd(D);

% extract the important parts of U, V, W
U = U(:, 1:3);
V = V(:, 1:3);
W = W(1:3, 1:3);

% calculate M and S matrices
M = U;
S = W * V';

% calculate L from M * L * M' = I
L = pinv(M) * eye(size(M, 1)) * pinv(M');

% calculate C using Cholesky factorization
C = chol(L,'lower');

% tune M and S
M = M*C;
S = pinv(C)*S;

% plot the camera positions
figure()
plot3(M(:, 1), M(:, 2), M(:, 3), '.')

% plot the resulting points contained in S
figure()
tri = delaunay(S(1, :), S(2, :));
trisurf(tri, S(1, :), S(2, :), S(3, :));
hold on
plot3(S(1, :), S(2, :), S(3, :), '.')
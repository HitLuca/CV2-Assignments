clear;

pwm = importdata('PointViewMatrix.txt');
% imshow(pwm, []);

pwm = center_points(pwm);

[U, W, V] = svd(pwm);

U = U(:, 1:3);
W = W(1:3, 1:3);
V = V(:, 1:3);

D = U * W * V';

M = U * W^(1/2);
S = W^(1/2) * V';

figure()
% plot3(M(:, 1), M(:, 2), M(:, 3), '.')
% hold on
plot3(S(1, :), S(2, :), S(3, :), '.')





% method = 'all';
% k = 1000;
% 
% for i=1:size(D, 1)
%     i
%     
%     % Merge Two Frames
%     [R, t] = ICP(source, target, NaN, k, method);
%     Frame_X_Y = transformPoints(source, R, t);
%     
%     % merge the merged frame with the last frame
%     if i==1
%         Frame_X_Y_Z = Frame_X_Y;
%     else
%         % run ICP and merge the frames
%        [R, t] = ICP(Frame_X_Y, Frame_X_Y_Z, NaN, k, method);
%        Frame_X_Y_Z =(Frame_X_Y_Z - t) *pinv(R'); 
%     end
%     
%     Frame_X_Y_Z = [Frame_X_Y_Z; Frame_X_Y];
% end
% 
% % plot of the point cloud but with reduction in number of points for more clarity
% figure
% plot3(Frame_X_Y_Z_2(:,1), Frame_X_Y_Z_2(:,2), Frame_X_Y_Z_2(:,3),'.', 'MarkerSize', 3);
% axis equal

%%

function [pwm] = center_points(pwm)
    for row = 1:size(pwm, 1)
        pwm(row, :) = pwm(row, :) - mean(pwm(row, :));
    end
end
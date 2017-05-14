% take frames and descriptors of image 1 and 2 and find the matches p1-p2
function [p1, p2] = match_images(f1, d1, f2, d2)
    % calculate matches using vl_ubcmatch
    matches = vl_ubcmatch(d1, d2, 2);

    p1 = [f1(1:2, matches(1,:)); ones(1, size(matches, 2))];
    p2 = [f2(1:2, matches(2,:)); ones(1, size(matches, 2))];
end


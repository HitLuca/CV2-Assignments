function [p1, p2] = match_images(image1, image2, n_matches)
    [f1, d1] = vl_sift(image1);
    [f2, d2] = vl_sift(image2);

    [matches, distances] = vl_ubcmatch(d1, d2);
    [~, indexes] = sort(distances);
    matches = matches(:, indexes);
    matches = matches(:, 1:n_matches);

    p1 = [f1(1:2, matches(1,:)); ones(1, n_matches)];
    p2 = [f2(1:2, matches(2,:)); ones(1, n_matches)];
end


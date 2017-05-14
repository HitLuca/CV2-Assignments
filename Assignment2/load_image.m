% load image given the index
function [image] = load_image(source_path, im_index)
    if im_index < 10    
        image = single(imread([source_path, '0', num2str(im_index), '.png']));
    else
        image = single(imread([source_path, num2str(im_index), '.png']));
    end
end
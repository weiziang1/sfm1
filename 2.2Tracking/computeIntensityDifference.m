%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:Given two images and the window center in both images,
% aligns the images wrt the window and computes the difference 
% between the two overlaid images.
% input: x1, y1(center of window in image1)
%        x2, y2(center of window in image2)
%        width,height(size of window)
% ouput: imgdiff(It)

function imgdiff = computeIntensityDifference(image1, image2, x1, y1, x2, y2, width, height)
    imgdiff = zeros(height,width);
    for i = -floor(width/2):floor(width/2)
        for j = -floor(height/2):floor(height/2)
            g1 = interpolate(x1+i, y1+j, image1);
            g2 = interpolate(x2+i, y2+j, image2);
            imgdiff(j+floor(height/2)+1, i+floor(width/2)+1) = g1 - g2;
        end
    end
end
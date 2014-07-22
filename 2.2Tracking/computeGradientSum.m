%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Given two gradients and the window center in both images,
% aligns the gradients wrt the window and computes the sum of the two
% overlaid gradients.
% input: gradx1, grady1(gradient image1)
%        gradx2, grady2(gradient image2)
%        x1, y1(center of window in image1)
%        x2, y2(center of window in image2)
%        width,height(window size)
% ouput: gradx,grady(gradient in window)

function [gradx, grady] = computeGradientSum(gradx1, grady1, gradx2, grady2, x1, y1, x2, y2, width, height)
    gradx = zeros(height,width);
    grady = zeros(height,width);
    for i = -floor(width/2):floor(width/2)
        for j = -floor(height/2):floor(height/2)
            g1 = interpolate(x1+i, y1+j, gradx1);
            g2 = interpolate(x2+i, y2+j, gradx2);
            gradx(j+floor(height/2)+1, i+floor(width/2)+1) = g1 + g2;
            g1 = interpolate(x1+i, y1+j, grady1);
            g2 = interpolate(x2+i, y2+j, grady2);
            grady(j+floor(height/2)+1, i+floor(width/2)+1) = g1 + g2;
        end
    end
end
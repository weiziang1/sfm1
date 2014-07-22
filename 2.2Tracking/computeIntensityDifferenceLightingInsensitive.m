%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Given two images and the window center in both images,
% aligns the images wrt the window and computes the difference
% between the two overlaid images; normalizes for overall gain and bias.
% input: x1, y1(center of window in image1)
%        x2, y2(center of window in image2)
%        width,height(size of window)
% ouput: imgdiff(It, normalized)

function [imgdiff, outofborder] = computeIntensityDifferenceLightingInsensitive(image1, image2, x1, y1, x2, y2, width, height)
    imgdiff = zeros(height,width);
    outofborder = 0;
    sum1 = 0; sum2 = 0;
    sum1_squared = 0; sum2_squared =0;
    for i = -floor(width/2):floor(width/2)
        for j = -floor(height/2):floor(height/2)
            if x1+i <= 1 || y1+j <= 1 || x1+i >= size(image1,2)-1 || y1+j >= size(image1,1)-1 
                outofborder = 1;
                break; 
            end
            if x2+i <= 1 || y2+j <= 1 || x2+i >= size(image1,2)-1 || y2+j >= size(image1,1)-1 || isnan(x2+i) || isnan(y2+j)
                outofborder = 1;
                break; 
            end
            g1 = interpolate(x1+i, y1+j, image1);
            g2 = interpolate(x2+i, y2+j, image2);
            sum1 = sum1 + g1; sum2 = sum2 + g2;
            sum1_squared = sum1_squared + g1*g1;
            sum2_squared = sum2_squared + g2*g2;
        end
    end
    mean1 = double(sum1_squared / (width*height));
    mean2 = double(sum2_squared / (width*height));
    alpha = sqrt(mean1/mean2);
    mean1 = sum1 / (width*height);
    mean2 = sum2 / (width*height);
    beta = mean1 - alpha*mean2;
    for i = -floor(width/2):floor(width/2)
        for j = -floor(height/2):floor(height/2)
            if x1+i <= 1 || y1+j <= 1 || x1+i >= size(image1,2)-1 || y1+j >= size(image1,1)-1 
                outofborder = 1;
                break; 
            end
            if x2+i <= 1 || y2+j <= 1 || x2+i >= size(image1,2)-1 || y2+j >= size(image1,1)-1 || isnan(x2+i) || isnan(y2+j)
                outofborder = 1;
                break; 
            end
            g1 = interpolate(x1+i, y1+j, image1);
            g2 = interpolate(x2+i, y2+j, image2);
            imgdiff(j+floor(height/2)+1, i+floor(width/2)+1) = g1 - g2*alpha - beta;
        end
    end
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: This file is used to track Keypoints to track referenced as 
% 'Detection and Tracking of Point Features'. For a single X,Y location,
% use the gradient Ix, Iy, and images im0, im1 to compute the new location.
% Here it may be necessary to interpolate Ix, Iy, im0, im1 if the
% corresponding locations are not integral. 
% Note: Discard any track move out of the image frame,
%       for a single X, Y location.
% input: a single X, Y location, the gradients Ix, Iy, and images im0, im1
% output: one iterative result of one point
% example:

function [keypoint_x2, keypoint_y2] = predictTranslation(keypoint_x1, keypoint_y1, gradx1, grady1, gradx2, grady2, image1, image2)
    iteration = 1; 
    keypoint_x2 = keypoint_x1;
    keypoint_y2 = keypoint_y1;
    while 1
        % window size 15x15
        [imgdiff, outofborder_1] = computeIntensityDifferenceLightingInsensitive(image1, image2, keypoint_x1,...
            keypoint_y1, keypoint_x2, keypoint_y2, 15, 15);
        [gradx, grady, outofborder_2] = computeGradientSumLightingInsensitive(gradx1, grady1, gradx2, grady2,...
            image1, image2, keypoint_x1, keypoint_y1, keypoint_x2,...
            keypoint_y2, 15, 15);
        [gxx, gxy, gyy] = compute2by2GradientMatrix(gradx, grady, 15, 15);
        [ex, ey] = compute2by1ErrorVector(imgdiff, gradx ,grady, 15, 15, 2.0);
        [dx, dy] = solveEquation(gxx, gxy, gyy, ex, ey);
        keypoint_x2 = keypoint_x2 + dx;
        keypoint_y2 = keypoint_y2 + dy;
        iteration = iteration + 1;
        if keypoint_x2 <= 1 || keypoint_y2 <= 1 || keypoint_x2 >= size(image1,2)-1 || keypoint_y2 >= size(image1,1)-1 || isnan(keypoint_y2) || isnan(keypoint_x2) || outofborder_1 || outofborder_2
            keypoint_x2 = 1; keypoint_y2 = 1;
            break;
        end
        if iteration > 20 || (abs(dx) < 0.01 && abs(dy) < 0.01) break; end
    end
end

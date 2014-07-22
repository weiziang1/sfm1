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
% example: image1 = imread('../hw4_supp/images/hotel.seq00.png');
%          imwrite(image1,'warp','gif', 'Loopcount',inf);
%          image2 = imread('../hw4_supp/images/hotel.seq50.png');
%          imwrite(image2,'warp','gif','WriteMode','append');
%          flag_imgshow = 1;

function [keypoints_x2, keypoints_y2] = predictTranslationAll(keypoints_x1, keypoints_y1, image1, image2, flag_imgshow)
    gx = [-1, 0, 1];
    gy = [-1, 0 ,1]';
    gradx1 = filter2(gx, image1, 'same');
    grady1 = filter2(gy, image1, 'same');
    gradx2 = filter2(gx, image2, 'same');
    grady2 = filter2(gy, image2, 'same');
  
    keypoints_y2 = keypoints_y1;
    keypoints_x2 = keypoints_x1;
    
    if flag_imgshow == 1
        figure(1); imshow(image1); hold on;
        plot(keypoints_x1, keypoints_y1, 'go', 'MarkerSize', 4.0);
        figure(2); imshow(image1); hold on;
    end
    
    for index = 1:size(keypoints_x1,2)
        [keypoint_x2, keypoint_y2] = predictTranslation(keypoints_x1(index), keypoints_y1(index),...
            gradx1, grady1, gradx2, grady2, image1, image2);
        keypoints_x2(index) = keypoint_x2;
        keypoints_y2(index) = keypoint_y2;
        if keypoint_x2 == 1 && keypoint_y2 == 1 && flag_imgshow == 1
             plot(keypoints_x1(index), keypoints_y1(index), 'ro', 'MarkerSize', 5.0);
        else
%             plot(keypoints_x2(index), keypoints_y2(index), 'bo');
%             line([keypoints_x1(index) keypoints_x2(index)], [keypoints_y1(index) keypoints_y2(index)], 'Color','r');
        end
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: This file is used to capture Keypoints to track. We use 
% Harris critia, and Shi-Tomasi/Kanade-Tomasi criteria respectively.
% Supposed that M is the second moment matrix, lambda1, lambda2 are the
% eigenvalues of M, and tau is the threshold for selecting keypoints: 
%           det(M)-alpha*trace(M)^2 geq tau
%           min(lambda1, lambda2) geq tau
% If using the Harris criteria, it is good to choose alpha in (0.01, 0.06].
% Choose tau so that edges and noisy patches are ignored.
% Then do local non-maxima suppression over a 5x5 window centered at each
% point.
% input: image, tau (threshold)
% output: matrix of keypoints
% example:
% image = imread('../hw4_supp/images/hotel.seq00.png');
% alpha =0.06;
% [keypoints_x1, keypoints_y1] = getkeypoints(image, alpha, 2);
% keypoints_x1 = keypoints_x1';
% keypoints_y1 = keypoints_y1';

function [posx, posy] = getkeypoints(image, alpha, method)
    %% Method 1, Harris criteria
    if method == 1
        gx = [-1, 0, 1];
        gy = [-1, 0 ,1]';
        X = filter2(gx, image, 'same');
        Y = filter2(gy, image, 'same');
        A = X.*X;
        B = Y.*Y;
        C = X.*Y;
        w = fspecial('gaussian', [5, 5], 0.5);
        A = imfilter(A, w);
        B = imfilter(B, w);
        C = imfilter(C, w);

        % construct M matrix
        R = zeros(size(image,1), size(image,2));
        lambda = zeros(size(image,1), size(image,2),2);
        Rmax = 0;
        for i = 1:size(image,1) % height
            for j = 1:size(image,2) % width
                M = [A(i,j) C(i,j); C(i,j) B(i,j)];
                R(i,j) = det(M) - alpha*trace(M)*trace(M);
                lambda(i,j,:) = eig(M);
                if R(i,j) > Rmax
                    Rmax = R(i, j);
                end
            end
        end

        % local non-maxima suppression, window size 5x5
        tau = 0.01 * Rmax;
        result = zeros(size(image,1), size(image,2));
        for i = 3:size(image,1)-2
            for j = 3:size(image,2)-2
                temp = R(i-2:i+2, j-2:j+2);
                loc_max = max(temp(:));
                if R(i, j) == loc_max && R(i,j) > tau
                    result(i,j) = 1;
                end
            end
        end

        % draw keypoints
        [posy, posx] = find(result == 1);
        figure(1),imshow(image);
        hold on;
        for ind = 1:size(posx, 1)
            plot(posx(ind), posy(ind), 'go', 'MarkerSize', 100*abs(R(posy(ind),posx(ind)))/Rmax);
        end
    else 
    %% Method 2, Shi-Tomasi
        gx = [-1, 0, 1];
        gy = [-1, 0 ,1]';
        X = filter2(gx, image, 'same');
        Y = filter2(gy, image, 'same');
        A = X.*X;
        B = Y.*Y;
        C = X.*Y;
        w = fspecial('gaussian', [5, 5], 0.5);
        A = imfilter(A, w);
        B = imfilter(B, w);
        C = imfilter(C, w);

        % construct M matrix
        R = zeros(size(image,1), size(image,2));
        lambda = zeros(size(image,1), size(image,2),2);
        Rmax = 0;
        for i = 1:size(image,1) % height
            for j = 1:size(image,2) % width
                M = [A(i,j) C(i,j); C(i,j) B(i,j)];
                R(i,j) = det(M) - alpha*trace(M)*trace(M);
                lambda(i,j,:) = eig(M);
                if R(i,j) > Rmax
                    Rmax = R(i, j);
                end
            end
        end

        % local non-maxima suppression, window size 5x5
        tau = 0.0001 * Rmax;
        result = zeros(size(image,1), size(image,2));
        for i = 3:size(image,1)-2
            for j = 3:size(image,2)-2
                temp = R(i-2:i+2, j-2:j+2);
                loc_max = max(temp(:));
                if R(i, j) == loc_max && R(i,j) && min(lambda(i,j,:)) >= tau
                    result(i,j) = 1;
                end
            end
        end

        % draw keypoints
        [posy, posx] = find(result == 1);
        figure(1),imshow(image);
        hold on;
        for ind = 1:size(posx, 1)
            plot(posx(ind), posy(ind), 'go', 'MarkerSize', 100*abs(R(posy(ind),posx(ind)))/Rmax);
        end
    end

end
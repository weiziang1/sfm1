%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo to track 50 frames

% load all.mat;

load('../hw4_supp/tracked_points.mat');
addpath('../hw4_supp/images');
ImgfileDir='../hw4_supp/images/';  %color
vistail= '.png';
frames=dir([ImgfileDir  '*' vistail] );
nFrame=length(frames);
image1 = imread(frames(1,1).name);

%% load from 2.1 keypoint selection
addpath('../2.1KeypointSelection');
alpha =0.06;
[keypoints_x1, keypoints_y1] = getkeypoints(image1, alpha, 2);
keypoints_x1 = keypoints_x1';
keypoints_y1 = keypoints_y1';

%% tracking 
x_keypoints_our = zeros(nFrame,size(keypoints_x1,2));
y_keypoints_our = zeros(nFrame,size(keypoints_x1,2));
x_keypoints_our(1,:) = keypoints_x1;
y_keypoints_our(1,:) = keypoints_y1;

for index = 2:51
    index/nFrame
    image1 = double(imread(frames(index-1,1).name));
    image2 = double(imread(frames(index,1).name));
    [keypoints_x2, keypoints_y2] = predictTranslationAll(keypoints_x1, keypoints_y1, image1, image2, 0);
    x_keypoints_our(index, :) = keypoints_x2;
    y_keypoints_our(index, :) = keypoints_y2;
    keypoints_x1 = keypoints_x2;
    keypoints_y1 = keypoints_y2;
end
save all.mat;

%% note: data structure above has been saved as allmat
load all.mat;

%% save outside points
image1 = imread(frames(1,1).name);
figure; imshow(image1); hold on;
[row, column] = find(y_keypoints_our == 1);
plot(x_keypoints_our(1,column), y_keypoints_our(1,column), 'r.', 'MarkerSize', 12.0);
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', [0,0, 512/100.0, 480/100.0]); 
set(gca,'position',[0 0 1 1]);  %remove the boundary
print(gcf,'-dpng', '-r100', ['outside.png'] );
save tracked_keypoints_my.mat x_keypoints_our y_keypoints_our;

%% make warp.gif
image = imread(frames(1,1).name);
figure(1); imshow(image); hold on;
plot(x_keypoints_our(1,:), y_keypoints_our(1,:), 'b.','MarkerSize', 12.0);
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', [0,0, 512/100.0, 480/100.0]); 
set(gca,'position',[0 0 1 1]);  %remove the boundary
print(gcf,'-dpng', '-r100', ['1.png'] );
[image,map]=rgb2ind(imread('1.png'),256);
imwrite(image,map,'warp','gif','DelayTime',1, 'Loopcount',inf);

for index = 11:10:51
    image = imread(frames(index,1).name);
    figure(1); imshow(image); hold on;
    plot(x_keypoints_our(index,:), y_keypoints_our(index,:), 'g.','MarkerSize', 12.0);
    set(gcf, 'PaperUnits', 'inches');
    set(gcf, 'PaperPosition', [0,0, 512/100.0, 480/100.0]); 
    set(gca,'position',[0 0 1 1]);  %remove the boundary
    print(gcf,'-dpng', '-r100', ['1.png'] );
    [image,map]=rgb2ind(imread('1.png'),256);
    imwrite(image,map,'warp','gif','DelayTime',1,'WriteMode','append');
end

%% make 20 random paths
num_without_outside = setdiff(1:size(y_keypoints_our,2),column);
index = round(rand(1, 20) * size(num_without_outside, 2));
color = rand(20, 3);
num_without_outside_sample = num_without_outside(:, index);
figure; imshow(image1); hold on;
for j=1:size(num_without_outside_sample,2)
    plot(x_keypoints_our(1:size(x_keypoints_our,1), num_without_outside_sample(1,j)),...
        y_keypoints_our(1:size(x_keypoints_our,1), num_without_outside_sample(1,j)),...
        'LineStyle', '-', 'Color', color(j, :), 'LineWidth', 5.0) ;
end
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperPosition', [0,0, 512/100.0, 480/100.0]); 
set(gca,'position',[0 0 1 1]);  %remove the boundary
print(gcf,'-dpng', '-r100', ['random20paths.png'] );
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Construct 2x2 Gradient Matrix as equation solver left-side
% input: gradx, grady(gradient within window)
%        width, height(window size)
% output: gxx, gxy, gyy (Sum the square gradient within window)

function [gxx, gxy, gyy] = compute2by2GradientMatrix(gradx, grady, width, height)
    gxx = 0; gxy = 0; gyy = 0;
    for i = 1:width
        for j = 1:height
            gx = gradx(j, i);
            gy = grady(j, i);
            gxx = gxx + gx*gx;
            gxy = gxy + gx*gy;
            gyy = gyy + gy*gy;
        end
    end
end
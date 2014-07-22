%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Construct 2x1 Gradient Matrix as equation solver right-side
% input: imgdiff(It within window)
%        gradx,grady(Ix, Iy within window)
%        width, height(window size)
%        step_factor(2.0 comes from equations, 1.0 seems to avoid overshooting)
% output: ex, ey(sum Ix*It and Iy*It within window respectively)

function [ex, ey] = compute2by1ErrorVector(imgdiff, gradx, grady, width, height, step_factor)
    ex = 0; ey = 0;
    for i = 1:width
        for j =1:height
            diff = imgdiff(j, i);
            ex = ex + diff*(gradx(j, i));
            ey = ey + diff*(grady(j, i));
        end
    end
    ex = step_factor * ex;
    ey = step_factor * ey;
end
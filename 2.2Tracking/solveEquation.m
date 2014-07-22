%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: Solves the 2x2 matrix equation
%           [gxx gxy] [dx] = [ex]
%           [gxy gyy] [dy] = [ey]
%            for dx and dy.
% input: gxx, gxy, gyy(left-side)
%        ex, ey(right-side)
% output: dx, dy(motion bias)

function [dx, dy] = solveEquation(gxx, gxy, gyy, ex, ey)
    det = gxx*gyy - gxy*gxy;
    dx = (gyy*ex - gxy*ey) / det;
    dy = (gxx*ey - gxy*ex) / det;
end
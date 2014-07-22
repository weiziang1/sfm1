%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x,y are float variable, we should interpolate the value at pos(x,y) in 
% the image, using (int(x), int(y)),(int(x)+1, int(y)),(int(x), int(y)+1),
% (int(x)+1, int(y)+1) to interpolate the value.

function value = interpolate(x, y, image)
    int_x = round(x);
    int_y = round(y);
    ax = x - int_x;
    ay = y - int_y;
    
    nw = image(int_y, int_x); % northwest
    ne = image(int_y+1, int_x); % northeast
    sw = image(int_y, int_x+1); % southwest
    se = image(int_y+1, int_x+1); % southeast
    
    value = (1-ax)*(1-ay)*nw + ...
        ax*(1-ay)*ne + ...
        (1-ax)*ay*sw + ...
        ax*ay*se;
end
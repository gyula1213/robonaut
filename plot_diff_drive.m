function plot_diff_drive(q, r_wheel,len, d, maxy, pixel_size,fotengely,delta,u)
% PLOT_DIFF_DRIVE(q, r_wheel, l_axis, len, maxy, pixel_size)
%  Drawing the (triangle-shaped) robot on an image
% Inputs:
%   q = [x,y,theta]: configuration of the robot
%   r_wheel: radius of wheels
%   l_axis: length of the axis (distance of the wheels)
%   len: length of the robot 
%   maxy: height of the image
%   pixel_size: size of pixels on the map (square-shaped) 

% Robot configuration
x = q(1);
y = q(2);
theta = q(3);
wheel_width = 5;

xbl = x - sin(theta) * len - fotengely*cos(theta); % balhátsó kerék középpont
ybl = y + cos(theta) * len - fotengely*sin(theta);
xbr = x + sin(theta) * len - fotengely*cos(theta); % jobbhátsó kerék középpont
ybr = y - cos(theta) * len - fotengely*sin(theta);
% Position of front left wheel
xl = x - sin(theta) * len;
yl = y + cos(theta) * len;

% Position of front right wheel
xr = x + sin(theta) * len;
yr = y - cos(theta) * len;
% Bycicle modell front wheel
xbyc = x;
ybyc = y;
hold on;

% Front of the robot
xe = x + d * cos(theta);
ye = y + d * sin(theta);

% Back of the robot
xb = x - d * cos(theta)- fotengely*cos(theta);
yb= y - d * sin(theta)- fotengely*sin(theta);


% Drawing the back triangle of the robot (triangle)
plot([xbr, xb, xbl, xbr]/pixel_size, [maxy-ybr, maxy-yb, maxy-ybl, maxy-ybr]/pixel_size, 'g', 'LineWidth',3);
patch('XData', [xbr, xb, xbl, xbr]/pixel_size, 'YData', [maxy-ybr, maxy-yb, maxy-ybl, maxy-ybr]/pixel_size, 'FaceColor', [0 0.2 1], 'EdgeColor', [0 0 1], 'FaceAlpha', .3, 'LineWidth', 3);
axis equal;
plot([xbl xbr]/pixel_size, ...
    [maxy-ybl maxy-ybr]/pixel_size, ...
    'b', 'LineWidth', 3);

% Drawing the front body of the robot (triangle)
plot([xr, xe, xl, xr]/pixel_size, [maxy-yr, maxy-ye, maxy-yl, maxy-yr]/pixel_size, 'g', 'LineWidth',3);
patch('XData', [xr, xe, xl, xr]/pixel_size, 'YData', [maxy-yr, maxy-ye, maxy-yl, maxy-yr]/pixel_size, 'FaceColor', [0 1 1], 'EdgeColor', [0 0 1], 'FaceAlpha', .3, 'LineWidth', 3);
axis equal;

% Drawing the fotengely
plot([x-fotengely*cos(theta) x]/pixel_size, ...
    [maxy-y+fotengely*sin(theta) maxy-y]/pixel_size, ...
    'b', 'LineWidth', wheel_width);

% Drawing velocity
% plot([x-fotengely*cos(theta) x-fotengely*cos(theta)+u*cos(theta)*2]/pixel_size, ...
%     [maxy-y+fotengely*sin(theta) maxy-y+fotengely*sin(theta)-u*sin(theta)*2]/pixel_size, ...
%     'r', 'LineWidth', wheel_width);

% Drawing the front wheels
plot([xl+r_wheel*cos(theta) xl-r_wheel*cos(theta)]/pixel_size, ...
    [maxy-(yl+r_wheel*sin(theta)) maxy-(yl-r_wheel*sin(theta))]/pixel_size, ...
    'k', 'LineWidth', wheel_width);
plot([xr+r_wheel*cos(theta) xr-r_wheel*cos(theta)]/pixel_size, ...
    [maxy-(yr+r_wheel*sin(theta)) maxy-(yr-r_wheel*sin(theta))]/pixel_size, ...
    'k', 'LineWidth', wheel_width);
% Drawing the bycicle front wheels
plot([xbyc+r_wheel*cos(theta+delta) xbyc-r_wheel*cos(theta+delta)]/pixel_size, ...
    [maxy-(ybyc+r_wheel*sin(theta+delta)) maxy-(ybyc-r_wheel*sin(theta+delta))]/pixel_size, ...
    'k', 'LineWidth', wheel_width);
% drawing back wheels
plot([xbl+r_wheel*cos(theta) xbl-r_wheel*cos(theta)]/pixel_size, ...
    [maxy-(ybl+r_wheel*sin(theta)) maxy-(ybl-r_wheel*sin(theta))]/pixel_size, ...
    'k', 'LineWidth', wheel_width);
plot([xbr+r_wheel*cos(theta) xbr-r_wheel*cos(theta)]/pixel_size, ...
    [maxy-(ybr+r_wheel*sin(theta)) maxy-(ybr-r_wheel*sin(theta))]/pixel_size, ...
    'k', 'LineWidth', wheel_width);
% Plotting the reference point of the robot
plot(x/pixel_size, (maxy-y)/pixel_size, 'r*');

% plot([xl xr]/pixel_size, [maxy-yl maxy-yr]/pixel_size, 'm*');
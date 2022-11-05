function q = diff_drive(q0, u, phi, l_mainaxis, dT)
% q = DIFF_DRIVE(q0, ,u,phi, l_mainaxis, dT):
%  Calculating the next state of the bycicle drive robot
% Outputs: 
%   q = [x; y; theta]: next position and orientation of the robot
% Inputs: 
%   q0 = [x0; y0; theta0]: initial state
%   u: velocity
%   phi: steering angle
%   l_mainaxis: length of axis (distance of rear axis and front axis)
%   dT: sampling time
% Initial state
x0 = q0(1);
y0 = q0(2); 
theta0 = q0(3);
% a világkoordináta rendszer közepe az első keréktengely közepén
% helyezkedik el 
d_theta = u/l_mainaxis*tan(phi)*dT;
theta = theta0 + d_theta;
d_x = u*cos(theta+phi)/cos(phi)*dT;
d_y = u*sin(theta+phi)/cos(phi)*dT;
% Next state
x = x0 + d_x; 
y = y0 + d_y;
q = [x; y; theta];
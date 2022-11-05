function [p_e, e] = line_sensing(q_e, no_of_sensors, dist, map, pixel_size)
% [p_e, e] = LINE_SENSING(q_e, no_of_sensors, dist, map, pixel_size)
%  Simulating the functioning of the sensors
% Output:
%   p_e: positions of the sensors (size(p_e) = (2, no_of_sensors)
%   e: vector of sensed values (rowvector)
% Inputs:
%   q_e = [x_e; y_e; theta_e]: position of the midpoint and orientation of the sensor
%   no_of_sensors: number of sensors
%   dist: distance between the sensors
%   map: map of the workspace of the robot (matrix)
%   pixel_size: width and height of one pixel on the map

% Vector for sensed values
e = zeros(1, no_of_sensors);

% Positions of the sensors
p_e = zeros(2, no_of_sensors);

for i = 1:no_of_sensors
    % Distance between the sensor and the midpoint
    dist_from_mid = ((no_of_sensors+1)/2 - i) * dist;
    p_e(1, i) = q_e(1) - dist_from_mid * cos(q_e(3));
    p_e(2, i) = q_e(2) - dist_from_mid * sin(q_e(3));
    
    % Index of pixel on the map
    map_x = round(p_e(1, i)/pixel_size);
    map_y = size(map,1)-round(p_e(2, i)/pixel_size);
    
    % Sensed value
    if map_x > 0 && map_y > 0 && map_x <= size(map,2) && map_y <= size(map,1)
        % Point inside of the map
        e(i) = map(map_y, map_x);
    else
        % Point outside of the map
        e(i) = 0;
    end
end
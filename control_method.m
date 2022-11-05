function [u, phi, p, p1] = control_method(q0, p1, e, no_sensors, l_mainaxis, u,phi,dist_sensors)
% [u, phi] = CONTROL_METHOD(q0, p_e, e, no_sensors, l_mainaxis, u,phi,dist_sensors)
%  Calculation of control inputs based on sensed values
% Inputs:
%   p_e: positions of sensors
%   e: sensed values
%   u: longitudinal velocity
%   l_axis: length of axis (distance of wheels)
%   r_wheel: radius of wheels
%   p : vonalérzékelési pont 
kp = -4/l_mainaxis/u;
count = 0;
szum = 0;
for i=1:no_sensors
        if (e(i)>0.6) 
            count = count + 1;
            szum = szum + i-(ceil(no_sensors/2));          %no_senz 32 esetén -16...15 ig
        end
end
if (count~=0)
    p1 =szum/count;
end
p = dist_sensors*p1;
phi = kp*p;




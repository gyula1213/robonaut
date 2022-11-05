% Simulation and control of a line follower robot
clear all; close all; clc;

%% Parameters of the robot

% Radius of a wheel
r_wheel = 2;
% Length between front and rear axis
l_mainaxis = 8;
% Length of the front and rear axis
len = 4;

% Length between front axis and line sensors
d = 1;
% virtuális vonalszenzorba helyezett kormányszög
gamma = 0;
%L' = l_mainaxis + d
% tan gamma= tan phi * l_mainaxis+d/l_mainaxis
% l_mainaxis és phi be a módosított értékeket kell majd beletenni (L', gamma) 

%Kezdo sebeseg és kormanyszog
u=3; % start velocity
phi = 0; %start steering angle

%Vonal orientáció
delta = 0;
% Vonal érzékelés pont 
p = 0;
p_start=0;
p_dif = 0;
p_t =0;
p1= 0;
p2= 0;
s_dif = 0;
delta_dif=0;
delta_fok_dif=0;
delta_fok_ket=0;
delta_ket=0;


% két vonal szenzor távolsága
senz_dist = 2*d + l_mainaxis; 

% Initial configuration of the robot: q0 = [x0; y0; theta0]
q0 = [10; 10; pi/2]; 

%% Sensors
no_sensors = 32; 
% Distance between two sensors
dist_sensors = 0.2;
 
%megtett út
s = 0;
%érzékelt vonalak száma
line_count = 0

%% Map of the environment

% Map from file
map = imread('maps/hullam.bmp');
map = 1 - map;

% Plotting the original map
figure(1);
imshow(1-map);
title('Original, noiseless map')

% Generating map with noise
deviation = 0.05;
new_map = map_with_noise(map, deviation);

% Size of one pixel
pixel_size = 0.1;  

%% Parameters of the simulation

% Sampling time [sec]
dT = 0.1;
% End time [sec]
T_end = 300;

% Save the results in a video file?
save_as_video = 0; % 0 - no, 1 - yes
% If yes, name of the file
v_filename = 'videos/xxx.avi';

%% Simulation

% Timevector for simulation
t = 0:dT:T_end;

% Storing the path of the robot ([x, y] coordinates in each row)
robot_path = zeros(length(t),2);
p_time = zeros(length(t),1);
phi_time = zeros(length(t),1);
[xplotmax ,yplotmax]=size(map);
y_plot = 5:yplotmax/20:yplotmax;

% Opening video file, if required
if save_as_video
    v = VideoWriter(v_filename);
    open(v)
end

% Loop of simulation
for i=1:length(t)
    % Storing the actual position in the path vector
    robot_path(i,:) = [q0(1) q0(2)];

    % Positions of the first sensors: midpoint of sensor and orientation
    q_e = [q0(1)+d*cos(q0(3)); q0(2)+d*sin(q0(3)); q0(3)-pi/2];
    % Positions of each sensors and sensed values
    [p_e, e] = line_sensing(q_e, no_sensors, dist_sensors, new_map, pixel_size);  
    [line_count] = vonalak_szama(e, no_sensors, dist_sensors);

    % Positions of the second sensors: midpoint of sensor and orientation
    q_e_b = [q0(1)-l_mainaxis*cos(q0(3))-d*cos(q0(3)); q0(2)-l_mainaxis*sin(q0(3))-d*sin(q0(3)); q0(3)-pi/2];
    % Positions of each sensors and sensed values
    [p_e_b, e_b] = line_sensing(q_e_b, no_sensors, dist_sensors, new_map, pixel_size); 
    
    % Plotting
    figure(2); hold off;
    % Plotting the map
    imshow(1-new_map);
    hold on;
    % Plotting the path
    plot(robot_path(1:i,1)/pixel_size, size(map,1)-robot_path(1:i,2)/pixel_size,'y:', 'LineWidth', 2);
    % Plotting the robot
    plot_diff_drive(q0, r_wheel, len, d ,size(map,1)*pixel_size, pixel_size,l_mainaxis,phi,u)
    % Plotting sensors
    for k = 1:size(p_e,2)
        plot(p_e(1,k)/pixel_size, (size(map,1)*pixel_size-p_e(2,k))/pixel_size, ...
            '.', 'MarkerSize', 15, 'Color', [e(k), (1-e(k))*0.8, 0]);
        plot(p_e_b(1,k)/pixel_size, (size(map,1)*pixel_size-p_e_b(2,k))/pixel_size, ...
            '.', 'MarkerSize', 15, 'Color', [e_b(k), (1-e_b(k))*0.8, 0]);
    end
    texts = [strcat("x= ",num2str(q0(1))),strcat("y= ",num2str(q0(2))), ...
        strcat("theta= ",num2str(q0(3)*180/pi)," fok"), strcat('velocity = ', num2str(u,2), ' m/s'),...
        strcat('phi = ', num2str(phi*180/pi), ' fok'),strcat('delta = ', num2str(delta_fok_ket,2), ' fok'),...
        strcat('ido = ', num2str((i-1)*dT,5), '/', num2str(T_end)),strcat('delta = ', num2str(delta_fok_ket,2), ' fok'),...
        strcat('p = ', num2str(p_t,2)),strcat('vonalak szama = ', num2str(line_count))];
    for k=1:length(texts)
        text(10,y_plot(k),texts(k));
    end
    % Control of the robot
    % Velocity of the robot
      
    title(["AUToBOT"]);
    
    
    
      
    % TASK: to implement the control strategy
    
    [u, phi, p_t,p1] = control_method(q0, p1, e, no_sensors, l_mainaxis, u, phi,dist_sensors);
    p_time(i,:)=p_t;
    phi_time(i,:)=phi;
    % TASK: to determine the next state of the robot
    q0 = diff_drive(q0, u, phi, l_mainaxis, dT);

    %vonal orientáció számitása
    %[delta, p, s, p_start] = vonal_orientacio_szamitas(e,p,no_sensors,u,dT,delta, p_e, s, p_start);

    %[delta_dif, p_dif, s_dif] = vonal_orientacio_szamitas_dif(e,p_dif,no_sensors,u,dT,delta_dif, p_e, s_dif);
    
    %[delta_ket, p1, p2] = vonal_orientacio_ket_szenz(e,p1,p2,no_sensors,delta_ket,e_b,senz_dist,dist_sensors);
    
%     delta_fok = delta/pi*180;
%     delta_fokok(i)=delta_fok;
%     delta_fok_dif = delta_dif/pi*180;
%     delta_fokok_dif(i)=delta_fok_dif;
%     delta_fok_ket = delta_ket/pi*180;
%     delta_fokok_ket(i)=delta_fok_ket;
    % Pausing simulation
    if i == 1 ||i == length(t) && ~save_as_video 
        title('Press any key to continue');
        pause; % Stop at the first step of iteration to resize the figure window if required
    else
        pause(dT)  ;
    end
    % Write current frame into video file
    if save_as_video
        cur_frame = getframe;
        if size(cur_frame.cdata,1) > size(map, 1) || size(cur_frame.cdata, 2) > size(map, 2)
            cur_frame.cdata = cur_frame.cdata(1:size(map,1), 1:size(map,2), :);
        end
        writeVideo(v,cur_frame);
    end
  
end
figure(2); hold off;
stairs(p_time);
hold on;
stairs(phi_time)
title('Tengelyközéppont beállása vonalugrás gerjesztés. kc szabályzás')
xlabel('time')
ylabel('p és phi')
legend({'p','phi'},'Location','southwest')
% hold on
% plot(delta_fokok_dif)
if save_as_video
    close(v);
end
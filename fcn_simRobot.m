function IR_distance = fcn_simRobot(torque_L,torque_R,sensorx,sensory,sensor_angle_in_degrees,varargin)
% fcn_simRobot
% This is a function written in support of simRobotWallFollowing that
% contains kinematic simulations, drawing code, and sensor simulations.

% Create a persistant structure called "robot" that is saved each time this
% function is called. The first time the function is called, the default
% values of the robot are filled in including the robot's size, the arena,
% etc.

persistent robot;

if isempty(robot)
    robot.width        = 10;
    robot.length       = 12;
    robot.radius_squared = robot.width.^2 + robot.length^2;
    robot.wheel_center = 4;
    robot.wheel_length = 3;
    robot.wheel_width  = 2;
    robot.position_x   = 0;
    robot.position_y   = -20;
    robot.theta        = 0;  % Units are radians
    robot.r_wheelangle = 0;
    robot.l_wheelangle = 0;
    robot.sensorx      = sensorx;  % Units are cm from midpoint between wheels
    robot.sensory      = sensory;  % Units are cm from midpoint between wheels
    robot.sensor_angle = sensor_angle_in_degrees*pi/180;  % Units are radians
    robot.sensor_range = 100;  % Sensor range: units are cm
    robot.omega_L = 0; % Initial value of robot's left motor speed
    robot.omega_R = 0; % Initial value of robot's right motor speed
    robot.delta_t = 0.01; % Time step for 100 Hz sampling rate
    robot.plot_every = 1; % Plot every 1 time steps
    robot.current_plot_index = 1; % This is the current index of plotting
    robot.start_time = cputime;
    figure(1);
    clf;
    fcn_drawRobot(robot);  % Draw the first time
end

%% Check the inputs
if nargin>5
    robot.plot_every = max(round(varargin{1}),1);
end

%% Constrain the torques to 100 percent limits
torque_L = min(torque_L,1);
torque_L = max(torque_L,-1);
torque_R = min(torque_R,1);
torque_R = max(torque_R,-1);


%% Euler simulation of kinematics

% Define sum of torques
torque_coulomb_L = 0.01; % Assume small percent of torque is colomb
torque_coulomb_R = 0.01; % Assume small percent of torque is colomb
max_speed = 20; % Units are cm/second
max_rotational_velocity = (max_speed / (robot.wheel_length/2)); % Max rotational speed of motor, in rad/sec

% Left motor calculations from torque to angular acceleration
J_L = .02;
damping_L = 1 / max_rotational_velocity; % Assume 100% of torque (1) is required to get to max velocity
torque_friction_L = torque_coulomb_L * sign(robot.omega_L)   + robot.omega_L*damping_L;
torque_net_L = torque_L - torque_friction_L;
rot_accel_L = torque_net_L / J_L;

% Left motor calculations from torque to angular acceleration
J_R = .02;
damping_R = 1.05 / max_rotational_velocity; % Assume 100% of torque (1) is required to get to max velocity
torque_friction_R = torque_coulomb_R * sign(robot.omega_R)  + robot.omega_R*damping_R;
torque_net_R = torque_R - torque_friction_R;
rot_accel_R = torque_net_R / J_R;

% From sum(Torque) = rotational_inertia * angular_acceleration
robot.omega_L = robot.omega_L + rot_accel_L*robot.delta_t;  % Units are cm/s
robot.omega_R = robot.omega_R + rot_accel_R*robot.delta_t;  % Units are cm/s

% Linear velocities
robot.v_L = robot.omega_L*robot.wheel_length/2;
robot.v_R = robot.omega_R*robot.wheel_length/2;

% Total velocity
V = (robot.v_L + robot.v_R)/2;
robot.omega_Robot = (robot.v_R - robot.v_L)/robot.width;

% Update the robot based on kinematics
robot.l_wheelangle = robot.l_wheelangle + robot.omega_L*robot.delta_t; % Agular speed of left wheel (rad/sec)
robot.r_wheelangle = robot.r_wheelangle + robot.omega_R*robot.delta_t;  % Angular speed of wheel (rad/sec)
robot.theta = robot.theta + robot.omega_Robot*robot.delta_t; % Angular speed of robot (rad/sec)
robot.position_x = robot.position_x + V*cos(robot.theta)*robot.delta_t;
robot.position_y = robot.position_y + V*sin(robot.theta)*robot.delta_t;

% Plot result by calling the drawRobot function, which also passes out the
% simulated IR distance
IR_distance = fcn_drawRobot(robot,1);  % Redraw the robot in Figure 1

% Keep track of number of times this function has been called
robot.current_plot_index = robot.current_plot_index + 1;

% Is the distance bad? If yes, exit.
if isnan(IR_distance)
    return;
end

% Is it time to redraw? If so, then redraw
if mod(robot.current_plot_index,robot.plot_every)==0
    drawnow;
end

end


function IR_distance = fcn_drawRobot(robot,varargin)
% fcn_drawRobot draws the robot
% Syntax:
% fcn_drawRobot(robot,varargin)
% Examples:
%      
%    % BASIC example - find all the points
%      
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
%

persistent arena;

if isempty(arena)
    % Specify the arena details
    %length_arena = 400;
    arena.wall_start = [-50 -50; 50 -50; 150 -150; 200 -150; 200 -200; 250 -200; 250 0; 200 0; 200 50; 250 50; 250 150; 0 250; 0 50; -50 0];
    arena.wall_end   = [arena.wall_start(2:end,:); arena.wall_start(1,:)];
    N_walls = length(arena.wall_start(:,1));
    
    arena.walls_x = [arena.wall_start(:,1) arena.wall_end(:,1) NaN*arena.wall_start(:,1)];
    arena.walls_y = [arena.wall_start(:,2) arena.wall_end(:,2) NaN*arena.wall_start(:,2)];
    arena.walls_x = reshape(arena.walls_x',N_walls*3,1);
    arena.walls_y = reshape(arena.walls_y',N_walls*3,1);
    
    
    %% Calculations to determine robot dimensions
    left_side  = -robot.width/2;
    right_side =  robot.width/2;
    front_side =  robot.wheel_center;
    rear_side  =  robot.wheel_center - robot.length;
    
    arena.body = [left_side front_side; right_side front_side; right_side rear_side; left_side rear_side];
    arena.right_wheel = [...
        right_side-robot.wheel_width/2  robot.wheel_length/2;
        right_side+robot.wheel_width/2  robot.wheel_length/2;
        right_side+robot.wheel_width/2 -robot.wheel_length/2;
        right_side-robot.wheel_width/2 -robot.wheel_length/2];
    arena.left_wheel = [...
        left_side-robot.wheel_width/2  robot.wheel_length/2;
        left_side+robot.wheel_width/2  robot.wheel_length/2;
        left_side+robot.wheel_width/2 -robot.wheel_length/2;
        left_side-robot.wheel_width/2 -robot.wheel_length/2];
    
    
    % Define dimensions of sensor
    sensor_width = 1.5;
    sensor_length = 0.5;
    
    % Create basic sensor box
    arena.sensor_box = [...
        -sensor_width/2  sensor_length/2;
        +sensor_width/2  sensor_length/2;
        +sensor_width/2 -sensor_length/2;
        -sensor_width/2 -sensor_length/2];
    
    % Project the sensor via a unit vector
    unit_sensor_vector = [0 1];
    
    
    % Rotate it by the sensor angle and translate sensor into place
    sensor_offset = 0;
    R = [cos(robot.sensor_angle - sensor_offset) sin(robot.sensor_angle - sensor_offset);...
        -sin(robot.sensor_angle - sensor_offset)  cos(robot.sensor_angle - sensor_offset)];
    arena.sensor_box = arena.sensor_box*R + [robot.sensorx robot.sensory];
    unit_sensor_vector = unit_sensor_vector*R;
    arena.sensor_vector = [robot.sensorx robot.sensory;...
        unit_sensor_vector*robot.sensor_range+[robot.sensorx robot.sensory]];
    
    % Draw spokes?
    arena.spoke_interval = 60*pi/180;
    arena.base_spoke_angles = (0:arena.spoke_interval:2*pi)';
  
    arena.N_spokes = length(arena.base_spoke_angles);
    arena.robot_right_spoke_widths_start = (right_side-robot.wheel_width/2)*ones(arena.N_spokes,1);
    arena.robot_right_spoke_widths_end   = (right_side+robot.wheel_width/2)*ones(arena.N_spokes,1);
    arena.robot_left_spoke_widths_start  = (left_side-robot.wheel_width/2)*ones(arena.N_spokes,1);
    arena.robot_left_spoke_widths_end    = (left_side+robot.wheel_width/2)*ones(arena.N_spokes,1);
    
    % Define the target locations
    arena.targets_start = [50 -40; 140 -135; 165 -145; 205 -150; 230 -195; 245 -140; 200  -5; 195 55; 230 55; 240 145; 110 195; 10 225; 10 50;   -40 0; -10 -45];
    arena.targets_end  = [50 -20; 155 -120; 165 -125; 225 -150; 230 -180; 225 -140; 170 -20; 185 65; 230 70; 220 135;  90 175; 30 205; 30 50; -20 -15; -10 -25];
    arena.targets_midpoints = (arena.targets_start + arena.targets_end)/2;
    
    arena.N_targets = length(arena.targets_start(:,1));
    arena.target_was_hit = zeros(arena.N_targets,1);
    
    arena.targets_x = [arena.targets_start(:,1) arena.targets_end(:,1) NaN*arena.targets_start(:,1)];
    arena.targets_y = [arena.targets_start(:,2) arena.targets_end(:,2) NaN*arena.targets_start(:,2)];
    arena.targets_x = reshape(arena.targets_x',arena.N_targets*3,1);
    arena.targets_y = reshape(arena.targets_y',arena.N_targets*3,1);
    
    % Saved traces of robot position    
    arena.robot_positions = NaN*ones(50000,2);
    
end


%% Set up for debugging
do_debug = 0; % Flag to plot the results for debugging
 
if do_debug    
    fig_num = 2; %#ok<UNRCH>
    figure(fig_num);
    flag_make_new_plot = 1;
end

%% check input arguments
if nargin < 1 || nargin > 3
    error('Incorrect number of input arguments.')
end

flag_make_new_plot = 0; % Default is not to make a new plot
if 2<= nargin
    fig_num = varargin{1};
    % figure(fig_num);  % This might not be needed - plot commands go to it
    % automatically
else
    fig = gcf; % create new figure with next default index
    fig_num = get(fig,'Number');
    flag_make_new_plot = 1;
end


%% Move the spokes?
r_spoke_angles = arena.base_spoke_angles - robot.r_wheelangle;
l_spoke_angles = arena.base_spoke_angles - robot.l_wheelangle;

r_spoke_y = robot.wheel_length/2*cos(r_spoke_angles);
l_spoke_y = robot.wheel_length/2*cos(l_spoke_angles);

r_spokes = [arena.robot_right_spoke_widths_start, r_spoke_y, ...
    arena.robot_right_spoke_widths_end, r_spoke_y];
l_spokes = [ arena.robot_left_spoke_widths_start, l_spoke_y,...
    arena.robot_left_spoke_widths_end, l_spoke_y];

%% Rotate the robot to current theta
offset = pi/2;
R = [cos(robot.theta - offset) sin(robot.theta - offset);...
    -sin(robot.theta - offset)  cos(robot.theta - offset)];
   
body = arena.body*R;
right_wheel = arena.right_wheel*R;
left_wheel  = arena.left_wheel*R;
sensor_box = arena.sensor_box*R;
sensor_vector = arena.sensor_vector*R;
r_spokes = [r_spokes(:,1:2)*R, r_spokes(:,3:4)*R];
l_spokes = [l_spokes(:,1:2)*R, l_spokes(:,3:4)*R];

%% Translate the robot to current XY coordinates
body = body + [robot.position_x robot.position_y];
right_wheel = right_wheel + [robot.position_x robot.position_y];
left_wheel  = left_wheel + [robot.position_x robot.position_y];
sensor_box = sensor_box + [robot.position_x robot.position_y];
sensor_vector = sensor_vector + [robot.position_x robot.position_y];
r_spokes = r_spokes + [robot.position_x robot.position_y robot.position_x robot.position_y];
l_spokes = l_spokes + [robot.position_x robot.position_y robot.position_x robot.position_y];



%% Check where sensor vector hits the walls
[distance,location] = fcn_findSensorHit(arena.wall_start,arena.wall_end,sensor_vector);

% Check if there was a hit
if distance>0 && distance<robot.sensor_range
    sensor_vector(2,:)=location;
else
    distance = robot.sensor_range;
end

% Calculate the IR distance
IR_distance = fcn_emulateIRSensorDistance(distance);

%% Check if any of the robot is outside of the walls
flag_no_collision = 1;

% all_points = [body; right_wheel; left_wheel; sensor_box];
% if any(all_points(:,1)>300)
%     flag_no_collision = 0;
% elseif any(all_points(:,2)<-50)
%     flag_no_collision = 0;
% elseif any(all_points(:,2)>100)
%     flag_no_collision = 0;
% end    

% Check if any of the robot hits the walls
right_wheel_sensor = right_wheel(2:3,:);
left_wheel_sensor =  [left_wheel(1,:); left_wheel(4,:)];
body_front_sensor = body(1:2,:);
body_rear_sensor = body(3:4,:);

[distance_right_wheel,location_right_wheel] = fcn_findSensorHit(arena.wall_start,arena.wall_end,right_wheel_sensor);
[distance_left_wheel,location_left_wheel]   = fcn_findSensorHit(arena.wall_start,arena.wall_end,left_wheel_sensor);
[distance_body_front,location_body_front]   = fcn_findSensorHit(arena.wall_start,arena.wall_end,body_front_sensor);
[distance_body_rear,location_body_rear]     = fcn_findSensorHit(arena.wall_start,arena.wall_end,body_rear_sensor);

if ~isnan(distance_right_wheel)
    fprintf(1,'Impact detected on right wheel at location: x = %.2f, y = %.2f,\n',location_right_wheel(1,1),location_right_wheel(1,2));    
    flag_no_collision = 0;
end
if ~isnan(distance_left_wheel)
    fprintf(1,'Impact detected on left wheel at location: x = %.2f, y = %.2f,\n',location_left_wheel(1,1),location_left_wheel(1,2));    
    flag_no_collision = 0;
end
if ~isnan(distance_body_front)
    fprintf(1,'Impact detected on front of robot at location: x = %.2f, y = %.2f,\n',location_body_front(1,1),location_body_front(1,2));    
    flag_no_collision = 0;
end
if ~isnan(distance_body_rear)
    fprintf(1,'Impact detected on rear of robot at location: x = %.2f, y = %.2f,\n',location_body_rear(1,1),location_body_rear(1,2));    
    flag_no_collision = 0;
end


if flag_no_collision ==0
    IR_distance = NaN;
end

%% Check intersections with targets
distance_squared_to_targets = sum((arena.targets_midpoints - [robot.position_x robot.position_y]).^2,2);
[min_distance_squared,index_min] = min(distance_squared_to_targets);
if(min_distance_squared<robot.radius_squared)
    if arena.target_was_hit(index_min,1) == 0 % This is the first time it was found?
        arena.target_was_hit(index_min,1) = 1;
        plot([arena.targets_start(index_min,1) arena.targets_end(index_min,1)],...
            [arena.targets_start(index_min,2) arena.targets_end(index_min,2)],...
            'g','Linewidth',5);
    end
end

% Check to see if you won!
if all(arena.target_was_hit(:,1))
    fprintf(1,'Competition complete! Your simulated completion time is: %.2f seconds. Wall time simulation duration was: %.2f seconds.\n',(robot.current_plot_index*robot.delta_t), (cputime-robot.start_time));
    IR_distance = NaN;  % Force the sim to exit
end


% OLD METHOD: (slow)
% for i=1:arena.N_targets
%     [distance_target,~]   = fcn_findSensorHit(arena.targets_start(i,:),arena.targets_end(i,:),body_front_sensor);
%     if ~isnan(distance_target)
%         arena.target_was_hit(i,1) = 1;
%         plot([arena.targets_start(i,1) arena.targets_end(i,1)],...
%             [arena.targets_start(i,2) arena.targets_end(i,2)],...
%             'g','Linewidth',5);
%     end
% end

%% Save the robot position
arena.robot_positions(robot.current_plot_index,:) = [robot.position_x robot.position_y];


%% Plot input results
if flag_make_new_plot 
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    % Plot arena (first time)
    plot(arena.walls_x,arena.walls_y,'k','Linewidth',5);
    
    % Plot targets (first time)
    plot(arena.targets_x,arena.targets_y,'r','Linewidth',5);
    
    % Plot the robot parts the first time
    handles.h_robot_body = plot_box(0,0,body,'k');  % body
    handles.h_robot_rwheel = plot_box(0,0,right_wheel,'r');  % R Wheel
    handles.h_robot_lwheel = plot_box(0,0,left_wheel,'b');  % L Wheel
    handles.h_robot_sensor = plot_sensor(0,0,sensor_box,sensor_vector,'g');  % Sensor
    handles.h_robot_rspokes = plot_spokes(0,0,r_spokes,r_spoke_angles,'r'); % Rwheel spokes
    handles.h_robot_lspokes = plot_spokes(0,0,l_spokes,l_spoke_angles,'b'); % Lwheel spokes

    % Drop an ant-trail
    handles.h_robot_path = plot(arena.robot_positions(:,1),arena.robot_positions(:,2),'c.');

    
    set(fig_num,'UserData',handles)
else % Update the line positions
    if mod(robot.current_plot_index,robot.plot_every)==0
        handles = get(fig_num,'UserData');
        handles.h_robot_body = plot_box(1,handles.h_robot_body,body,'k');  % body
        handles.h_robot_rwheel = plot_box(1,handles.h_robot_rwheel,right_wheel,'r');  % R Wheel
        handles.h_robot_lwheel = plot_box(1,handles.h_robot_lwheel,left_wheel,'b');  % L Wheel
        handles.h_robot_sensor = plot_sensor(1,handles.h_robot_sensor,sensor_box,sensor_vector,'g');  % Sensor
        handles.h_robot_rspokes = plot_spokes(1,handles.h_robot_rspokes,r_spokes,r_spoke_angles,'r'); % Rwheel spokes
        handles.h_robot_lspokes = plot_spokes(1,handles.h_robot_lspokes,l_spokes,l_spoke_angles,'b'); % Lwheel spokes

        % Update the ant-trail
        set(handles.h_robot_path,'Xdata',arena.robot_positions(:,1));
        set(handles.h_robot_path,'Ydata',arena.robot_positions(:,2));
        
        set(fig_num,'UserData',handles)
    end
    
end

end % Ends the function





%% Functions for plotting

function h_box = plot_box(flag_plot_exists, h_box_old, corners, varargin)    % plot all the boxes

plot_str = 'b-';
plot_type = 1;

if 4 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

xdata = [corners(:,1); corners(1,1)];
ydata = [corners(:,2); corners(1,2)];

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_box = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_box = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_box_old,'XData',xdata);
    set(h_box_old,'YData',ydata);
    h_box = h_box_old;
end


end % Ends plot_box function


function h_sensor = plot_sensor(flag_plot_exists, h_box_old, corners, sensor_vector, varargin)    % plot sensor

plot_str = 'b-';
plot_type = 1;

if 5 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

xdata = [corners(:,1); corners(1,1); NaN; sensor_vector(:,1)];
ydata = [corners(:,2); corners(1,2); NaN; sensor_vector(:,2)];

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_sensor = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_sensor = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_box_old,'XData',xdata);
    set(h_box_old,'YData',ydata);
    h_sensor = h_box_old;
end


end % Ends plot_box function

function h_spokes = plot_spokes(flag_plot_exists, h_spokes_old, spokes, spoke_angles, varargin)    % plot all the spokes

plot_str = 'b-';
plot_type = 1;

if 5 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

% Tag hidden angles to hide them
spoke_angles = mod(spoke_angles,2*pi);
spokes(spoke_angles>pi,:) = NaN;


N_spokes = length(spokes(:,1));
xmatrix = [spokes(:,1), spokes(:,3), NaN*spokes(:,1)];

xdata = reshape(xmatrix',N_spokes*3,1);

ymatrix = [spokes(:,2), spokes(:,4), NaN*spokes(:,2)];
ydata = reshape(ymatrix',N_spokes*3,1);

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_spokes = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_spokes = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_spokes_old,'XData',xdata);
    set(h_spokes_old,'YData',ydata);
    h_spokes = h_spokes_old;
end


end % Ends plot_spokes function


function IR_distance = fcn_emulateIRSensorDistance(distance)   
% IR_distance = fcn_emulateIRSensorDistance(distance)
% Syntax:
% IR_distance = fcn_emulateIRSensorDistance(distance)
% Examples:
     
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
% 

persistent distance_array

if isempty(distance_array)
    % Convert distance
    numerator = 200*30;
    crossover = 6;  % Units are centimeters
    offset = 20;
    IR_distance_at_crossover = numerator./(crossover+offset);
   
    distance_array = (0:0.1:100)';
    distance_array = [distance_array zeros(length(distance_array(:,1)),1)];
    Npoints = length(distance_array(:,1));
    for i=1:Npoints
        distance_to_fill = distance_array(i,1);
        if distance_to_fill<crossover
            IR_distance = distance_to_fill/crossover * IR_distance_at_crossover;
        else
            IR_distance = numerator./(distance_to_fill+offset);
        end
        distance_array(i,2) = IR_distance;
    end
    
    for i=1:Npoints-10
        distance_array(i,2) = mean(distance_array(i:i+10,2));
    end
end
IR_distance = interp1(distance_array(:,1),distance_array(:,2),distance) + 3*randn(length(distance(:,1)),1);


end



function [distance,location] = fcn_findSensorHit(wall_start,wall_end,sensor_vector,varargin)   
% fcn_findSensorHit calculates hits between sensor vector and walls
% Syntax:
% fcn_findSensorHit(arena.wall_start,arena.wall_end,sensor_vector,varargin) 
% Examples:
%      
%    % BASIC example - find all the points
%      
% 
% This function was written on 2020_04_13 by S. Brennan
% Questions or comments? sbrennan@psu.edu 
% 
% Adopted from https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

%% Set up for debugging
do_debug = 0; % Flag to plot the results for debugging
 
if do_debug    
    fig_num = 23333;  %#ok<UNRCH>
    figure(fig_num);
    clf;
    hold on;
    axis equal;
    grid on; grid minor;
    
    N_walls = length(wall_start(:,1));
    walls_x = [wall_start(:,1) wall_end(:,1) NaN*wall_start(:,1)];
    walls_y = [wall_start(:,2) wall_end(:,2) NaN*wall_start(:,2)];
    walls_x = reshape(walls_x',N_walls*3,1);
    walls_y = reshape(walls_y',N_walls*3,1);
        
    plot(walls_x,walls_y,'k','Linewidth',1);
    plot(sensor_vector(:,1),sensor_vector(:,2),'g');
    
end
% 
% %% check input arguments
% if nargin < 1 || nargin > 2
%     error('Incorrect number of input arguments.')
% end
% 
% flag_make_new_plot = 0; % Default is not to make a new plot
% if 2 == nargin
%     fig_num = varargin{1};
%     figure(fig_num);
% else
%     fig = gcf; % create new figure with next default index
%     fig_num = get(fig,'Number');
%     flag_make_new_plot = 1;
% end



%% Calculations begin here
% Define r and s vectors
p = wall_start;
q = sensor_vector(1,:);
r = wall_end - wall_start;
s = sensor_vector(2,:)-sensor_vector(1,:);
r_cross_s = crossProduct(r,s);
q_minus_p =  q - p;

q_minus_p_cross_s = crossProduct(q_minus_p,s);
q_minus_p_cross_r = crossProduct(q_minus_p,r);

parallel_indices = find(0==r_cross_s);
if any(parallel_indices)
    r_cross_s(parallel_indices) = 1; % They are colinear or parallel, so make dummy length
end
   
t = q_minus_p_cross_s./r_cross_s;
u = q_minus_p_cross_r./r_cross_s;

t(parallel_indices) = inf;
u(parallel_indices) = inf;

intersection = NaN*ones(length(p(:,1)),2);

good_vector = ((0<t).*(1>t).*(0<u).*(1>u));
good_indices = find(good_vector>0);
if ~isempty(good_indices)
    result = p + t.*r; 
    intersection(good_indices,:) = result(good_indices,:);
    %plot(intersection(:,1),intersection(:,2),'rx');
end

distances_squared = sum((intersection - sensor_vector(1,:)).^2,2);
[best,best_index] = min(distances_squared);

distance = best^0.5;
location = intersection(best_index,:);
end




%% Calculate cross products
function result = crossProduct(v,w)
result = v(:,1).*w(:,2)-v(:,2).*w(:,1);
end






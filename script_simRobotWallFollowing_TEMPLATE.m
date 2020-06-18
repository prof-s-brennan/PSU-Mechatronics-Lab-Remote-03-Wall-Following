clear all;

% STUDENTS: change the following 3 lines of code to put your sensor where
% you want to!
sensorx      = 4;  % Units are cm from midpoint between wheels
sensory      = 4;  % Units are cm from midpoint between wheels
sensor_angle_in_degrees = -40;  % Units are degrees, with 0 degrees pointing straight forward


delta_t = 0.01; %100 Hz sampling rate
t = 0:delta_t:500;

IR_distance = 100; % Start off with a long-range reading

for i=1:length(t)
    time = t(i); % This is the current time
    
    % STUDENTS: Put your algorithm here to drive the robot following the
    % wall and passing through all the check markers. You get to change 
    % the speed_command torque and  turn torque based on IR_distance reading, 
    % and using this and some smarts, try to get the
    % robot complete task as fast as possible.

    speed_command = 0.8;   
    turn_command = -0.1; % Positive commands turn to the right        
    
    torque_L = speed_command + turn_command;
    torque_R = speed_command - turn_command;
    
    % Simulate robot and grab the IR distance
    IR_distance = fcn_simRobot(torque_L,torque_R,sensorx,sensory,sensor_angle_in_degrees,20);   
    
    title(sprintf('IR reading is: %.f',IR_distance));
    
    % Check to see if you slam into a wall: if so, the IR reading becomes
    % not-a-number, e.g. NaN!
    if isnan(IR_distance)
        break;
    end
    


end





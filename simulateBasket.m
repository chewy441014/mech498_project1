function simulateBasket(pos_ball, vel_ball)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize things
dt = 0.00009; % Time step
frames_per_second = 60; % For video output
robot = basketInit();
home_pos = robot.home_pos;
home_angles = robot.home_angles;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create ball trajectory
[is_solution,ball_traj] = ballTrajectory(pos_ball, vel_ball, robot, dt);
if is_solution == false
    error('Invalid ball trajectory')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find interception joint angles 
smallest_dist = 99999999;
intersect_dt = 0;
intersect_time = 0;
% Look at every point of ball trajectory and find the closest one to the
% robot home position
for i = 1:size(ball_traj,2)
    dist = norm(home_pos - ball_traj(:,i));
    if(dist < smallest_dist)
        smallest_dist = dist;
        intersect_dt = i;
        intersect_time = i*dt;
    end
end
t_f = intersect_time;
% Find the orientation of the ball when the robot intercepts it
tangent = ball_traj(:,intersect_dt-1) - ball_traj(:,intersect_dt);
theta_y = atan2(tangent(1),tangent(3));
theta_z = atan2(tangent(2),tangent(1));
Ry = [cos(theta_y) 0 sin(theta_y);
    0 1 0;
    -sin(theta_y) 0 cos(theta_y)];
Rz = [cos(theta_z) -sin(theta_z) 0;
    sin(theta_z) cos(theta_z) 0;
    0 0 1];
R = Rz*Ry;
T = R;
T(1:4,4) = [ball_traj(:,intersect_dt); 1];

% Calculate desired joint angles based on ball interception point and
% orientation
[isReachable, joint_angles] = basketIK(T, home_angles, robot);
if ~isReachable
    warning('Intercept point may not be within the workspace')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Moving to the ball interception location - PD control
disp('Moving to the ball interception location');

%Theta_ref is the intersection point
%Theta_init is the zero position of the robot
theta_init = [home_angles, zeros(5,1)]; %5*2 matrix, first column start joint angles
theta_ref = [joint_angles, zeros(5,1)]; %5*2 matrix, first column intersection point
time1 = 0:dt:t_f;

% Initially set the ball mass to 0 because the robot is not holding the
% ball yet
robot.ball.mass = 0;

K_p = [200 200 200 200 400];
K_v = [40 40 50 10 20];
[joint_angles_mat1,~] = controlBasketPID(theta_init, theta_ref,  K_p, K_v, time1, robot);
end_angles = joint_angles_mat1(:,end);

% Animation output settings
% number_of_frames1 = time1(end)*frames_per_second;
% skip_frames1 = round(length(time1)/number_of_frames1);
skip_frames1 = round(0.0167/dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Catching the ball and remaining stationary (Impulse Input)
disp('Catching the ball and remaining stationary');

% Final time is 5 seconds. Modify if necessary.
time2 = 0:dt:2.5;
theta_init2 = [end_angles, zeros(5,1)];

% Set the mass of the ball to 5kg because the robot is now holding the ball
robot.ball.mass = 5;

Kp = 150*ones(5,1);
Kv = 25*ones(5,1);
[joint_angles_mat2, ~] = controlBasketImpulse(theta_init2, Kp, Kv, robot, tangent, time2);
end_angles2 = joint_angles_mat2(:,end);

% Animation settings
% number_of_frames2 = time2(end)*frames_per_second;
% skip_frames2 = round(length(time2)/number_of_frames2);
skip_frames2 = round(0.0167/dt);

% Find the position of the ball from the simulation
disp('Calculating position of ball during impulse simulation');
n = length(time2);
ball_pos = zeros(3,n);
for i = 1:skip_frames2:length(time2)
    [T,~] = basketFK(joint_angles_mat2(:,i), robot);
    ball_pos(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Moving to the pre-dunk position
% disp('Moving to the pre-dunk position');
% 
% predunking = robot.goal.predunking; % Gets the position of the end effector in preparation for dunking
% T_prebasket = eye(4); T_prebasket(1:3,4) = predunking; % Assembles transformation matrix using pre-dunking end effector position
% [~, pre_dunk_angles] = basketIK(T_prebasket, zeros(5,1), robot);
% 
% time3 = 0:dt:2*t_f; % We are in no rush to get the robot over to the Pre-basket position, we can make this value higher if we want.
% theta_init2 = [end_angles2, zeros(5,1)];
% theta_ref = [pre_dunk_angles, zeros(5,1)];
% 
% K_p2 = 20*ones(5,1);
% K_v2 = 20*ones(5,1);
% fprintf('\n')
% disp('Control Basket PID')
% disp('      ')
% [joint_angles_mat3,~] = controlBasketPID(theta_init2, theta_ref, K_p2, K_v2, time3, robot);
% end_angles3 = joint_angles_mat3(:,end);
% 
% fprintf('\n')
% disp('Calculating new Ball Pos ')
% disp('      ')
% 
% number_of_frames3 = time3(end)*frames_per_second;
% skip_frames3 = round(length(time3)/number_of_frames3);
% 
% n = length(time3);
% ball_pos2 = zeros(3,n);
% for i = 1:skip_frames3:length(time3)
%     fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
%     [T,~] = basketFK(joint_angles_mat3(:,i), robot);
%     ball_pos2(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
% end
% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Move robot to predunking position and then dunk

[joint_angles_mat3, time3] = Dunking(end_angles2, dt);

% Animation settings
% number_of_frames3 = time3(end)*frames_per_second;
% skip_frames3 = round(length(time3)/number_of_frames3);
skip_frames3 = round(0.0167/dt);

disp('Calculating position of ball during dunking simulation')
n = length(time3);
ball_pos2 = zeros(3,n);
for i = 1:skip_frames3:length(time3)
    [T,~] = basketFK(joint_angles_mat3(:,i), robot);
    ball_pos2(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end
    
norm_vec = arrayfun(@(idx) norm(ball_pos2(:,idx)), 1:size(ball_pos2,2));
[~, col, ~] = find(norm_vec); %find only the nonzero terms
ball_pos3_extract = ball_pos2(:, col);

% post-dunk motion of the ball
dunk_ball_vel = (ball_pos3_extract(:, end) - ball_pos3_extract(:, end - 1))/(skip_frames3*dt);
dunk_ball_pos = ball_pos3_extract(:, end);
[bounce_trajectory, time4] = post_dunk_Trajectory(dunk_ball_vel, dunk_ball_pos, dt, robot);

% Animation settings
% number_of_frames4 = time4(end)*frames_per_second;
% skip_frames4 = round(length(time4)/number_of_frames4);
skip_frames4 = round(0.0167/dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw the robot

fprintf('\n')
prompt = 'Draw the robot? (Type int only, 1 = true, 0 = false) ';
draw = input(prompt);

while draw
    robot.handles = drawBasket(theta_init, pos_ball, robot);
    
    for t = 1:skip_frames1:length(time1)
        setBasket(joint_angles_mat1(:,t), t*dt, robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_traj(1,t));
        set(O, 'YData', ball_traj(2,t));
        set(O, 'ZData', ball_traj(3,t));
    end
    for t = 1:skip_frames2:length(time2)
        setBasket(joint_angles_mat2(:,t), t*dt, robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_pos(1,t));
        set(O, 'YData', ball_pos(2,t));
        set(O, 'ZData', ball_pos(3,t));
    end
    for t = 1:skip_frames3:length(time3)
        setBasket(joint_angles_mat3(:,t), t*dt, robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_pos2(1,t));
        set(O, 'YData', ball_pos2(2,t));
        set(O, 'ZData', ball_pos2(3,t));
    end
    for t = 1:skip_frames4:length(time4)
        setBasket(joint_angles_mat3(:,end), t*dt, robot);
        O = robot.handles(7).Children;
        set(O, 'XData', bounce_trajectory(1,t));
        set(O, 'YData', bounce_trajectory(2,t));
        set(O, 'ZData', bounce_trajectory(3,t));
    end
    
    prompt = 'Draw the robot again? (Type int only, 1 = true, 0 = false) ';
    draw = input(prompt);
end
disp('Done!');

end

function simulateBasket(pos_ball, vel_ball)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize things
dt = 0.0001; % Time step
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

K_p = [125 125 100 100 200];
K_v = [30 30 35 10 15];
[joint_angles_mat1,~] = controlBasketPID(theta_init, theta_ref,  K_p, K_v, time1, robot);
end_angles = joint_angles_mat1(:,end);

skip_frames = round(0.0225/dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Catching the ball and remaining stationary (Impulse Input)
disp('Catching the ball and remaining stationary');

% Final time is 5 seconds. Modify if necessary.
time2 = 0:dt:1.5;
theta_init2 = [end_angles, zeros(5,1)];

% Set the mass of the ball to 5kg because the robot is now holding the ball
robot.ball.mass = 2;

K_p = [100 100 100 100 200];
K_v = [40 40 35 20 30];

[joint_angles_mat2, ~] = controlBasketImpulse(theta_init2, K_p, K_v, robot, tangent, time2);
end_angles2 = joint_angles_mat2(:,end);

% Find the position of the ball from the simulation
disp('Calculating position of ball during impulse simulation');
n = length(time2);
ball_pos = zeros(3,n);
for i = 1:skip_frames:length(time2)
    [T,~] = basketFK(joint_angles_mat2(:,i), robot);
    ball_pos(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Move robot to predunking position and then dunk

[joint_angles_mat3, time3] = Dunking(end_angles2, dt);

disp('Calculating position of ball during dunking simulation')
n = length(time3);
ball_pos2 = zeros(3,n);
for i = 1:skip_frames:length(time3)
    [T,~] = basketFK(joint_angles_mat3(:,i), robot);
    ball_pos2(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end
    
norm_vec = arrayfun(@(idx) norm(ball_pos2(:,idx)), 1:size(ball_pos2,2));
[~, col, ~] = find(norm_vec); %find only the nonzero terms
ball_pos3_extract = ball_pos2(:, col);

% post-dunk motion of the ball
dunk_ball_vel = (ball_pos3_extract(:, end) - ball_pos3_extract(:, end - 1))/(skip_frames*dt);
dunk_ball_pos = ball_pos3_extract(:, end);
[bounce_trajectory, time4] = post_dunk_Trajectory(dunk_ball_vel, dunk_ball_pos, dt, robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Follow trajectory back to the home position

t_f = 5;
end_angles5 = joint_angles_mat3(:,end);
trajectory = createCelebrateTrajectory(end_angles5,dt,t_f,robot);
time5 = 0:dt:t_f;
joint_angles_mat5 = controlDunkPID([end_angles5, zeros(5,1)], trajectory, Kp, Kv, time5, robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw the robot

fprintf('\n')
prompt = 'Draw the robot? (Type int only, 1 = true, 0 = false) ';
draw = input(prompt);
% frame_time = 0.1;
while draw
    robotVideo = VideoWriter('robotVideo.avi');
    open(robotVideo);
    robot.handles = drawBasket(theta_init, pos_ball, robot);
    
    for t = 1:skip_frames:length(time1)
%         tic;
        setBasket(joint_angles_mat1(:,t), t*dt, 'Catching Ball', robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_traj(1,t));
        set(O, 'YData', ball_traj(2,t));
        set(O, 'ZData', ball_traj(3,t));
%         val = toc;
        writeVideo(robotVideo, getframe)
%         pause(frame_time - val)
    end
    passed_time = length(time1)*dt;
    for t = 1:skip_frames:length(time2)
%         tic;
        setBasket(joint_angles_mat2(:,t), passed_time + t*dt, 'Impulse Control', robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_pos(1,t));
        set(O, 'YData', ball_pos(2,t));
        set(O, 'ZData', ball_pos(3,t));
%         val = toc;
        writeVideo(robotVideo, getframe)
%         pause(frame_time - val)
    end
    passed_time = passed_time + length(time2)*dt;
    for t = 1:skip_frames:length(time3)
%         tic;
        setBasket(joint_angles_mat3(:,t), passed_time + t*dt, 'Dunking', robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_pos2(1,t));
        set(O, 'YData', ball_pos2(2,t));
        set(O, 'ZData', ball_pos2(3,t));
%         val = toc;
        writeVideo(robotVideo, getframe)
%         pause(frame_time - val)
    end
    passed_time = passed_time + length(time3)*dt;
    for t = 1:skip_frames1:length(time5)
%         tic;
        setBasket(joint_angles_mat5(:,t), t*dt, 'Ball Bouncing and Moving Home', robot);
        if time5(t) <= time4(end)
            O = robot.handles(7).Children;
            set(O, 'XData', ball_traj(1,t));
            set(O, 'YData', ball_traj(2,t));
            set(O, 'ZData', ball_traj(3,t));    
        end
        writeVideo(robotVideo, getframe)
%         val = toc;
%         pause(frame_time - val)
    end
    passed_time = passed_time + length(time4)*dt;
    setBasket(joint_angles_mat5(:,end), passed_time, 'Task Completed', robot);
    O = robot.handles(7).Children;
    set(O, 'XData', bounce_trajectory(1,end));
    set(O, 'YData', bounce_trajectory(2,end));
    set(O, 'ZData', bounce_trajectory(3,end));

    writeVideo(robotVideo, getframe)
    close(robotVideo);
    prompt = 'Draw the robot again? (Type int only, 1 = true, 0 = false) ';
    draw = input(prompt);
end
disp('Done!');

end

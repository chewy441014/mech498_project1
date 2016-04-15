robot = basketInit();
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;
theta_x = pi;
R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
T = R_x;
% T(1:4,4) = [l_3 0 l_2+l_4 1]';
T(1:4,4) = [robot.goal.pos; 1];

[~, joint_angles] = basketIK(T,zeros(5,1),robot);
drawBasket(joint_angles,robot)

%%
robot = basketInit();
pos_ball = [5 0 0];
vel_ball = [-2 0 10];
dt = 0.01;
[~, ball_trajectory] = ballTrajectory(pos_ball, vel_ball, robot, dt);
ball_trajectory;
robot.handles = drawBasket([0 0 0 0 0],robot);
F(size(ball_trajectory,2)) = struct('cdata',[],'colormap',[]);
robotVideo = VideoWriter('robotVideo.avi');
open(robotVideo);
for t = 1:size(ball_trajectory,2)
    setBasket([0 0 0 0 0],robot);
    O = robot.handles(7).Children;
    set(O, 'XData', ball_trajectory(1,t));
    set(O, 'YData', ball_trajectory(2,t));
    set(O, 'ZData', ball_trajectory(3,t));
    writeVideo(robotVideo, getframe)
end
close(robotVideo);

%%
robot = basketInit();

theta_x = 0;
R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
T = R_x;
T(1:4,4) = [2; 0.5; 2; 1];

[~, theta_ref] = basketIK(T,zeros(5,1),robot);

t = 0:0.0001:15;
theta_init = [0; 0; 0; 0; pi/8];
theta_init(:,2) = zeros(5,1);
% theta_ref = [pi/2; 0; 0; 0; pi/2];
theta_ref(:,2) = zeros(5,1);
Kp = 5*ones(5,1);
Kv = 15*ones(5,1);
[joint_pos, joint_vel] = controlBasketPID(theta_init, theta_ref, Kp, Kv, t, robot);

robot.handles = drawBasket(theta_init,robot);
for t = 1:500:length(t)
    setBasket(joint_pos(:,t),robot);
end
disp('Done!');


%%
clc
pos_ball = [5 -5 0];
vel_ball = [-2.5 3 12];
simulateBasket(pos_ball, vel_ball);

%%
clc
pos_ball = [5 0 0];
vel_ball = [-1.5 0 12];
simulateBasket(pos_ball, vel_ball);

%%
clc
pos_ball = [5 -5 0];
vel_ball = [-2.5 3 12];


dt = 0.00009; % Time step
robot = basketInit();
home_pos = robot.home_pos;
home_angles = robot.home_angles;
[is_solution,ball_traj] = ballTrajectory(pos_ball, vel_ball, robot, dt);
if is_solution == false
    error('Invalid ball trajectory')
end
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

drawBasket(joint_angles,[0; 0; 0],robot);
hold on;
scatter3(ball_traj(1,:),ball_traj(2,:),ball_traj(3,:),1)

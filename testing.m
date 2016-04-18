%%
robot = basketInit();
theta_ref2 = [pi/2; 0.15; -0.6; 0; 0];
% theta_ref2 = [0; 0; 0; 0; 0];
% T = eye(3);
% T(1:4,4) = [robot.goal.pos; 1];
T = basketFK(theta_ref2,robot);
T(3,4) = T(3,4) + robot.parameters.l_1;
[~,joint_angles] = basketIK(T,theta_ref2,robot);
drawBasket(joint_angles,[0;0;0],robot);

%%
robot = basketInit();
drawBasket([0 0 0 0 0],[0;0;0],robot);

%%
robot = basketInit();
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;
theta_x = pi/0.9;
R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
T = R_x;
T(1:4,4) = [robot.goal.pos; 1];

[~, joint_angles] = basketIK(T,zeros(5,1),robot);
drawBasket(joint_angles,[0; 0; 0],robot)

joint_angles

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
vel_ball = [-1.5 1 15];
simulateBasket(pos_ball, vel_ball);

%%
clc
pos_ball = [5 5 0];
vel_ball = [-2 -2.2 12];
simulateBasket(pos_ball, vel_ball);

%%
clc
pos_ball = [20 0 0];
vel_ball = [-12 -0.7 10];
simulateBasket(pos_ball, vel_ball);

%%
clc
pos_ball = [5 -5 0];
vel_ball = [-1.5 1 15];


dt = 0.001; % Time step
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
% theta_y = -atan2(tangent(3),tangent(1))
theta_y = -acos(tangent(1)/norm(tangent))
% theta_z = -atan2(tangent(3),tangent(2));
theta_x = atan2(tangent(3),tangent(2))
% theta_x = pi/6;
Ry = [cos(theta_y) 0 sin(theta_y);
    0 1 0;
    -sin(theta_y) 0 cos(theta_y)]
% Rz = [cos(theta_z) -sin(theta_z) 0;
%     sin(theta_z) cos(theta_z) 0;
%     0 0 1]
Rx = [1 0 0;
    0 cos(theta_x) -sin(theta_x);
    0 sin(theta_x) cos(theta_x)]
R = Ry*Rx

% b = tangent/norm(tangent);
% a = [1; 0; 0];
% v = cross(a,b);
% s = norm(v);
% c = dot(a,b);
% v_x = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
% R = eye(3) + v_x + v_x^2*(1-c)/s^2;

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

%%
robot = basketInit();
dt = 0.0005; % Time step
skip_frames = round(0.0225/dt);
theta_ref2 = [pi/2; 0.15; -0.6; 0; 0];
K_p = 0.001*[100 100 100 100 200];
K_v = 0.001*[40 40 35 20 30];

t_f = 5;
trajectory = createCelebrateTrajectory(theta_ref2,dt,t_f,robot);
time5 = 0:dt:t_f;
joint_angles_mat5 = controlDunkPID([theta_ref2, zeros(5,1)], trajectory, K_p, K_v, time5, robot);

robot.handles = drawBasket(theta_ref2, [0;0;0], robot);
for t = 1:skip_frames:length(time5)
        setBasket(joint_angles_mat5(:,t), t*dt, 'Ball Bouncing and Moving Home', robot);
end
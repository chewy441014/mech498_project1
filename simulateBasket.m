function simulateBasket(pos_ball, vel_ball)
dt = 0.00009;

robot = basketInit();

[is,ball_traj] = ballTrajectory(pos_ball, vel_ball, robot, dt);
if is == false
    disp('Correct the Ball Trajectory')
    pause(1);
end
home_pos = robot.home_pos;
home_angles = robot.home_angles;

smallest_dist = 99999999;
intersect_dt = 0;
intersect_time = 0;
for i = 1:size(ball_traj,2)
    dist = norm(home_pos - ball_traj(:,i));
    if(dist < smallest_dist)
        smallest_dist = dist;
        intersect_dt = i;
        intersect_time = i*dt;
    end
end
tangent = abs(ball_traj(:,intersect_dt) - ball_traj(:,intersect_dt-1));
theta_y = tan(tangent(1)/tangent(3));
theta_z = tan(tangent(2)/tangent(1));
Ry = [cos(theta_y) 0 sin(theta_y);
    0 1 0;
    -sin(theta_y) 0 cos(theta_y)];
Rz = [cos(theta_z) -sin(theta_z) 0;
    sin(theta_z) cos(theta_z) 0;
    0 0 1];
R = Rz*Ry;
T = R;
T(1:4,4) = [ball_traj(:,intersect_dt); 1];

t_f = intersect_time;

%Moving to the Ball Intersection Location - PID control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculating the intersection point joint angles 
[~, joint_angles] = basketIK(T, home_angles, robot);

theta_init = [home_angles, zeros(5,1)]; %5*2 matrix, first column start joint angles
theta_ref = [joint_angles, zeros(5,1)]; %5*2 matrix, first column intersection point
time1 = 0:dt:t_f;
robot.ball.mass = 0;
%Theta_ref is the intersection point
%Theta_init is the zero position of the robot

K_p = 20*ones(5,1);
K_v = 20*ones(5,1);
[joint_angles_mat1,~] = controlBasketPID(theta_init, theta_ref,  K_p, K_v, time1, robot);
end_angles = joint_angles_mat1(:,end);


%Catching the Ball and Remaining Stationary (Impulse Input)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time2 = 0:dt:5; %I have hard coded impulse to use five seconds. We'll see how that turns out. 
theta_init2 = [end_angles, zeros(5,1)];
robot.ball.mass = 5;

Kp = 20*ones(5,1);
Kv = 25*ones(5,1);
[joint_angles_mat2, ~] = controlBasketImpulse(theta_init2, Kp, Kv, robot, tangent, time2);
end_angles2 = joint_angles_mat2(:,end);

ball_pos = zeros(3,length(time2));
for i = 1:length(time2)
    [T,~] = basketFK(joint_angles_mat2(:,i), robot);
    ball_pos(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end


% t_f2=t_f; %time to move back, change if necessary

%Moving to the Pre-Basket Position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

predunking = robot.goal.predunking; %Gets the position of the end effector in preparation for dunking
T_prebasket = eye(4); T_prebasket(1:3,4) = predunking; %Assembles transformation matrix using pre-dunking end effector position
pre_basket_desired_angles = [0; 0; 0; 0; pi/2]; %We want get to the arm to align with the the expected joint angles (joint five must be pi/2)
[~, pre_dunk_angles] = basketIK(T_prebasket, pre_basket_desired_angles, robot);
if ~(round(pre_dunk_angles(5)/(pi/2),1) == 1)
    disp('Dunk will look like shit')
    pause(1);
end

time3 = 0:dt:2*t_f; %We are in no rush to get the robot over to the Pre-basket position, we can make this value higher if we want.
theta_init = [end_angles2, zeros(5,1)];
theta_ref = [pre_dunk_angles, zeros(5,1)];

K_p2 = 20*ones(5,1);
K_v2 = 20*ones(5,1);
[joint_angles_mat3,~] = controlBasketPID(theta_init, theta_ref, K_p2, K_v2, time3, robot);
end_angles3 = joint_angles_mat3(:,end);

robot.handles = drawBasket(theta_init,robot);
for t = 1:125:length(time1)
    setBasket(joint_angles_mat1(:,t),robot);
    O = robot.handles(7).Children;
    set(O, 'XData', ball_traj(1,t));
    set(O, 'YData', ball_traj(2,t));
    set(O, 'ZData', ball_traj(3,t));
end
for t = 1:125:length(time2)
    setBasket(joint_angles_mat2(:,t),robot);
    O = robot.handles(7).Children;
    set(O, 'XData', ball_pos(1,t));
    set(O, 'YData', ball_pos(2,t));
    set(O, 'ZData', ball_pos(3,t));
end
for t = 1:125:length(time3)
    setBasket(joint_angles_mat3(:,t),robot);
    O = robot.handles(7).Children;
    set(O, 'XData', ball_pos(1,t));
    set(O, 'YData', ball_pos(2,t));
    set(O, 'ZData', ball_pos(3,t));
end
disp('Done!');

return

%Control Law for Dunking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K_p3 = 4*ones(5,1);
K_v3 = 10*ones(5,1);

dunk_trajectory = createDunkTrajectory(end_angles3,dt,4*t_f);
theta_init = [end_angles3, zeros(5,1)];
time4 = 0:dt:4*t_f;

[joint_angles_mat4] = controlDunkPID(theta_init, dunk_trajectory, K_p3, K_v3, time4, robot);

%Draw the Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot.handles = drawBasket( home_angles, robot );
for i=1:length(time1)
    setBasket(joint_angles_mat1(:,i), robot)
end
for i=1:length(time2)
    setBasket(joint_angles_mat2(:,i), robot)
end
for i=1:length(time3)
    setBasket(joint_angles_mat3(:,i), robot)
end
for i=1:length(time4)
    setBasket(joint_angles_mat4(:,i), robot)
end

end
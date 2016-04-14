function simulateBasket(pos_ball, vel_ball)
dt = 0.00009;
frames_per_second = 60;

robot = basketInit();

[is,ball_traj] = ballTrajectory(pos_ball, vel_ball, robot, dt);
if is == false
    fprintf('\n')
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
disp('Control Basket PID')
disp('      ')
pause(0.5)
[joint_angles_mat1,~] = controlBasketPID(theta_init, theta_ref,  K_p, K_v, time1, robot);
end_angles = joint_angles_mat1(:,end);

number_of_frames1 = time1(end)*frames_per_second;
skip_frames1 = round(length(time1)/number_of_frames1);

%Catching the Ball and Remaining Stationary (Impulse Input)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time2 = 0:dt:5; %I have hard coded impulse to use five seconds. We'll see how that turns out. 
theta_init2 = [end_angles, zeros(5,1)];
robot.ball.mass = 5;

Kp = 20*ones(5,1);
Kv = 25*ones(5,1);
fprintf('\n')
disp('Control Basket Impulse')
disp('      ')
pause(0.5)
[joint_angles_mat2, ~] = controlBasketImpulse(theta_init2, Kp, Kv, robot, tangent, time2);
end_angles2 = joint_angles_mat2(:,end);

fprintf('\n')
disp('Calculating new Ball Pos ')
disp('      ')
pause(0.5)

number_of_frames2 = time2(end)*frames_per_second;
skip_frames2 = round(length(time2)/number_of_frames2);

n = length(time2);
ball_pos = zeros(3,n);
for i = 1:skip_frames2:length(time2)
    fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
    [T,~] = basketFK(joint_angles_mat2(:,i), robot);
    ball_pos(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end

%Moving to the Pre-Basket Position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

predunking = robot.goal.predunking; %Gets the position of the end effector in preparation for dunking
T_prebasket = eye(4); T_prebasket(1:3,4) = predunking; %Assembles transformation matrix using pre-dunking end effector position
[~, pre_dunk_angles] = basketIK(T_prebasket, zeros(5,1), robot);

time3 = 0:dt:2*t_f; %We are in no rush to get the robot over to the Pre-basket position, we can make this value higher if we want.
theta_init2 = [end_angles2, zeros(5,1)];
theta_ref = [pre_dunk_angles, zeros(5,1)];

K_p2 = 20*ones(5,1);
K_v2 = 20*ones(5,1);
fprintf('\n')
disp('Control Basket PID')
disp('      ')
pause(0.5)
[joint_angles_mat3,~] = controlBasketPID(theta_init2, theta_ref, K_p2, K_v2, time3, robot);
end_angles3 = joint_angles_mat3(:,end);

fprintf('\n')
disp('Calculating new Ball Pos ')
disp('      ')
pause(0.5)

number_of_frames3 = time3(end)*frames_per_second;
skip_frames3 = round(length(time3)/number_of_frames3);

n = length(time3);
ball_pos2 = zeros(3,n);
for i = 1:skip_frames3:length(time3)
    fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
    [T,~] = basketFK(joint_angles_mat3(:,i), robot);
    ball_pos2(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
end

%Control Law for Dunking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n')
prompt = 'Run the dunking code? (Type int only, 1 = true, 0 = false) ';
x = input(prompt);

if x
    K_p3 = 4*ones(5,1);
    K_v3 = 10*ones(5,1);

    dunk_trajectory = createDunkTrajectory(end_angles3,dt,50*t_f);
    theta_init = [end_angles3, zeros(5,1)];
    time4 = 0:dt:4*t_f;

    fprintf('\n')
    disp('Control Dunk PID')
    disp('      ')
    pause(0.5)
    [joint_angles_mat4] = controlDunkPID(theta_init, dunk_trajectory, K_p3, K_v3, time4, robot);

    fprintf('\n')
    disp('Calculating new Ball Pos ')
    disp('      ')
    pause(0.5)
    
    number_of_frames4 = time4(end)*frames_per_second;
    skip_frames4 = round(length(time4)/number_of_frames4);

    n = length(time4);
    ball_pos3 = zeros(3,n);
    for i = 1:skip_frames4:length(time4)
        fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
        [T,~] = basketFK(joint_angles_mat4(:,i), robot);
        ball_pos3(:,i) = T(1:3,4) + [0; 0; robot.parameters.l_1];
    end
end

%Draw the Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n')
prompt = 'Draw the robot? (Type int only, 1 = true, 0 = false) ';
x2 = input(prompt);

if x2
    robot.handles = drawBasket(theta_init,robot);
    for t = 1:skip_frames1:length(time1)
        setBasket(joint_angles_mat1(:,t),robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_traj(1,t));
        set(O, 'YData', ball_traj(2,t));
        set(O, 'ZData', ball_traj(3,t));
    end
    for t = 1:skip_frames2:length(time2)
        setBasket(joint_angles_mat2(:,t),robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_pos(1,t));
        set(O, 'YData', ball_pos(2,t));
        set(O, 'ZData', ball_pos(3,t));
    end
    for t = 1:skip_frames3:length(time3)
        setBasket(joint_angles_mat3(:,t),robot);
        O = robot.handles(7).Children;
        set(O, 'XData', ball_pos2(1,t));
        set(O, 'YData', ball_pos2(2,t));
        set(O, 'ZData', ball_pos2(3,t));
    end
    if x
        for t = 1:skip_frames4:length(time4)
            setBasket(joint_angles_mat4(:,t),robot);
            O = robot.handles(7).Children;
            set(O, 'XData', ball_pos3(1,t));
            set(O, 'YData', ball_pos3(2,t));
            set(O, 'ZData', ball_pos3(3,t));
        end
    end
    disp('Done!');
end

end
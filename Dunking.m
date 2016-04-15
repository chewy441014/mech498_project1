function [Dunking_trajectory] = Dunking(init_angles, dt)
robot = basketInit;
% %%%
% robot.handles = drawBasket([pi/2; 0.35; -0.8; 0; pi/2], [0,0,0], robot);
% setBasket([pi/2; 0.35; -0.8; 0; pi/2], robot);
% pause
% setBasket([pi/2; 0.35; -0.8; 0; 0], robot);
% %%%
theta_ref1 = [pi/2; 0.35; -0.8; 0; pi/2];
theta_ref1 = [theta_ref1 zeros(5,1)];
[T, ~] = basketFK(init_angles, robot);
init_pos = T(1:3, 4);
[T, ~] = basketFK(theta_ref1, robot);
pos_ref = T(1:3, 4);
distance = norm(abs(init_pos - pos_ref));
avg_vel = 1;
t_f = distance/avg_vel;
time1 = 0:dt:t_f;
K_p = 20*ones(1, 5);
K_v = 10*ones(1, 5);
init_angles = [init_angles zeros(5,1)];
disp('        ')
disp('Calculating trajectory to pre-dunking position')
disp('        ')
[joint_angles_mat1, joint_velocities_mat] = ...
    controlBasketPID(init_angles, theta_ref1, K_p, K_v, time1, robot);
robot.handles = drawBasket(robot.home_angles, [0,0,0], robot);

%throwing the ball
% %%%
% joint_angles_mat1 = [pi/2; 0.35; -0.8; 0; pi/2];
% theta_ref1 = [pi/2; 0.35; -0.8; 0; pi/2];
% theta_ref1 = [theta_ref1 zeros(5,1)];
% time1 = [];
% joint_angles_mat1 = [pi/2; 0.35; -0.8; 0; pi/2];
% %%%
theta_ref2 = [pi/2; 0.15; -0.6; 0; 0];
theta_dot_ref = [0; 0; 0; 0; -3];
theta_ref2 = [theta_ref2 theta_dot_ref];
[T, ~] = basketFK(theta_ref2, robot);
pos_ref = T(1:3, 4);
t_f = (theta_ref2(end) - theta_ref1(end))/(theta_dot_ref(end));
time2 = 0:dt:t_f;
K_p = 150*ones(1, 5);
K_v = 20*ones(1, 5);
init_angles = [theta_ref1 zeros(5,1)];
disp('      ')
disp('Calculating dunking trajectory')
disp('        ')
[joint_angles_mat2, joint_velocities_mat] = ...
    controlBasketPID(init_angles, theta_ref2, K_p, K_v, time2, robot);
disp('       ')
time = [time1 time2];
frames_per_second = 60;
number_of_frames = time(end)*frames_per_second;
skip_frames = round(length(time)/number_of_frames);
joint_angles_mat = [joint_angles_mat1 joint_angles_mat2];
robot.handles = drawBasket(robot.home_angles, [0,0,0], robot);
pause
for t = 1:skip_frames:length(time)
    setBasket(joint_angles_mat(:,t), robot);
end

end
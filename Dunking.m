function [joint_angles_mat, time] = Dunking(init_angles, dt)
robot = basketInit;
theta_ref1 = [pi/2; 0.35; -0.8; 0; pi/2];
theta_ref1 = [theta_ref1 zeros(5,1)];
[T, ~] = basketFK(init_angles, robot);
init_pos = T(1:3, 4);
[T, ~] = basketFK(theta_ref1, robot);
pos_ref = T(1:3, 4);
distance = norm(abs(init_pos - pos_ref));
avg_vel = 1;
t_f1 = distance/avg_vel;
time1 = 0:dt:t_f1;

K_p = [50 50 50 50 100];
K_v = [20 20 25 8 10];

init_angles = [init_angles zeros(5,1)];
disp('Calculating trajectory to pre-dunking position')
[joint_angles_mat1, joint_velocities_mat] = ...
    controlBasketPID(init_angles, theta_ref1, K_p, K_v, time1, robot);
%throwing the ball
theta_ref2 = [pi/2; 0.15; -0.6; 0; 0];
theta_dot_ref = [0; 0; 0; 0; -3];
theta_ref2 = [theta_ref2 theta_dot_ref];
[T, ~] = basketFK(theta_ref2, robot);
pos_ref = T(1:3, 4);
t_f = (theta_ref2(end) - theta_ref1(end))/(theta_dot_ref(end));
time2 = t_f1:dt:t_f1 + t_f;

K_p = [50 50 50 50 100];
K_v = [20 20 25 8 10];

init_angles = [[pi/2; 0.35; -0.8; 0; pi/2] zeros(5,1)];
disp('Calculating dunking trajectory')
[joint_angles_mat2, joint_velocities_mat] = ...
    controlBasketPID(init_angles, theta_ref2, K_p, K_v, time2, robot);
time = [time1 time2];
frames_per_second = 60;
joint_angles_mat = [joint_angles_mat1 joint_angles_mat2];

end
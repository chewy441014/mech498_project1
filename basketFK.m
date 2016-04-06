function [T, robot_T] = basketFK(joint_angles, robot)
%BASKETFK Returns 4x4 homogeneous transformation matrix of basketBot
%   Takes in 6-element vector of joint anlges and a robot
%   structure and outputs the full forward kinemetics transform matrix (T)
%   and a cell array transforms between each frame.

% Shorten joint angle variable names
t_1 = joint_angles(1);
t_2 = joint_angles(2);
t_3 = joint_angles(3);
t_4 = joint_angles(4);
t_5 = joint_angles(5);

% Shorten length variable names
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;

% Generate transforms between each frame
T_01 = dhtf(0, 0, 0, t_1);
T_12 = dhtf(pi/2, 0, 0, t_2 + pi/2);
T_23 = dhtf(0, l_2, 0, t_3);
T_34 = dhtf(pi/2, 0, l_3, t_4);
T_45 = dhtf(-pi/2, 0, 0, t_5 - pi/2);
T_56 = dhtf(0, l_4, 0, 0);

% Generate full transform matrix and cell array
T = T_01*T_12*T_23*T_34*T_45*T_56;
robot_T = {T_01, T_12, T_23, T_34, T_45, T_56};


end
function z401ManipulatorDesign()

clear all
close all
clc

robot = basketInit();

% robot = robotInit array

% Cross-sectional areas of each arm member
d1 = robot.parameters.d_1;
d2 = robot.parameters.d_2;
d3 = robot.parameters.d_3;
d4 = robot.parameters.d_4;

A1 = pi/4*d1^2;
A2 = pi/4*d2^2;
A3 = pi/4*d3^2;
A4 = pi/4*d4^2;

% Other parameters
m1 = robot.parameters.m_1;
m2 = robot.parameters.m_2;
m3 = robot.parameters.m_3;
m4 = robot.parameters.m_4;
m5 = 0;

g = robot.const.g;

l1 = robot.parameters.l_1;
l2 = robot.parameters.l_2;
l3 = robot.parameters.l_3;
l4 = robot.parameters.l_4;

% Test joint torques at the zero position
joint_angles = [0;0;0;0;0];
joint_vel = [0;0;0;0;0];

drawBasket(joint_angles,robot)

[forces, moments]=z401ForceFinder(joint_angles,[0;0;0;0;0],[0;0;0;0;0])

[~, ~, G] = basketDynamics(joint_angles, joint_vel, robot);
tau = G;


%% Member 4

%George's inferior code

F5 = m4*g + m5*g;
% x4 = [0, l4/2, l4];
% shear4 = [F5, F5-m4*g, 0];
% moment4 = [tau(5), tau(5)+F5*l4/2, 0];
% 
% stairs(x4,shear4);
% title('Member 4 Shear Diagram');
% xlabel('x (m)');
% ylabel('Shear (N)');
% 
% figure
% plot(x4,moment4);
% title('Member 4 Moment Diagram');
% xlabel('x (m)');
% ylabel('Moment (N-m)');

%Treating weight as a distributed load

x=linspace(0,l4,100);
w=m4*g/l4;
V4=w*(l4-x);
M4=-w/2*(l4-x).^2;

%Ymax=(-wl4^4)/(8EI)
T5=w*l4^2/2;
F1=w*l4;

figure
subplot(2,1,1)
plot(x,V4)
xlabel('x (m)');
ylabel('Shear (N)');

subplot(2,1,2)
plot(x,M4)
xlabel('x (m)');
ylabel('Moment (N-m)');

%% Member 3

F3=m3*g+F5;

T3=m3*g*l3/2+T5+F5*l3;

x1=linspace(0,l3/2,50);
V31=F3+0*x1;
M31=F3*x1-T3;

x2=linspace(l3/2,l3,50);
V32=F5+0*x2;
M32=-F5*(l3-x2)-T5;

figure
 
subplot(2,1,1)
plot([x1 x2],[V31 V32],'-')
xlabel('x (m)');
ylabel('Shear (N)');

subplot(2,1,2)
plot([x1 x2],[M31 M32])
xlabel('x (m)');
ylabel('Moment (N-m)');

%% Member 2

F2=F3+m2*g

T2=T3

%% Member 1

%Reaction forces at the ground
Rf=F2+m1*g
Rm=T2


return
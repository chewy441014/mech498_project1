function z401ManipulatorDesign()

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
[~, ~, G] = basketDynamics(joint_angles, joint_vel, robot);
tau = G

% Member 4

%George's inferior code

% F5 = m4*g + m5*g;
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
V=w*(l4-x);
M=-w/2*(l4-x).^2;
%Ymax=(-wl4^4)/(8EI)

plot(x,V)
xlabel('x (m)');
ylabel('Shear (N)');

figure
plot(x,M)
xlabel('x (m)');
ylabel('Moment (N-m)');

%Member 3







return
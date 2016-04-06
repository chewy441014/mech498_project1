function [M, V, G] = basketDynamics(joint_angles, joint_vel, robot)

%% Needed Parameters

%Angles
t1 = joint_angles(1);
t2s = joint_angles(2)+pi/2;
t3 = joint_angles(3);
t4 = joint_angles(4);
t5 = joint_angles(5);

%Robot parameters
g = robot.parameters.g;
m_1 = robot.parameters.m_1;
m_2 = robot.parameters.m_2;
m_3 = robot.parameters.m_3;
m_4 = robot.parameters.m_4;
m_5 = 0;

%this will need to change-> m_5 will vary depending on whether or not
%the robot is holding the ball
%m_5 = robot.parameters.m_5;

l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;

%% Building G Matrix

G1 = 0;

G2 = -m_2*l_2*g*sin(t2s)/2-m_3*g*l_2*sin(t2s)+m_3*g*l_3/2*cos(t2s+t3)...
    -m_4*g*l_2*sin(t2s)+m_4*g*l_3*cos(t2s+t3)+m_4*g*l_4/2*cos(t2s+t3+t5)...
    -m_5*g*l_2*sin(t2s)+m_5*g*l_3*cos(t2s+t3)+m_5*g*l_4*cos(t2s+t3+t5);

G3 = m_3*g*l_3/2*cos(t2s+t3)+m_4*g*l_3*cos(t2s+t3)+m_4*g*l_4/2*cos(t2s+t3+t5)...
    +m_5*g*l_3*cos(t2s+t3)+m_5*g*l_4*cos(t2s+t3+t5);

G4 = 0;

G5 = m_4*g*l_4/2*cos(t2s+t3+t5)+m_5*g*l_4*cos(t2s+t3+t5);

G=[G1;G2;G3;G4;G5];


end
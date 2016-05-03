function [forces, moments] = z401ForceFinder(the, d_the, dd_the)

robot = basketInit();

% %Solid links 
% % Link lengths (meters)
% l_2 = robot.parameters.l_2;
% l_3 = robot.parameters.l_3;
% l_4 = robot.parameters.l_4;
% 
% % Link diameters (meters)
% d_2 = robot.parameters.d_2;
% d_3 = robot.parameters.d_3;
% d_4 = robot.parameters.d_4;
% 
% % Material Properties
% rho = 2.7*10^3; % Aluminum, kg/m^3
% 
% % Link masses (kg)
% m1 = 0;
% m2 = pi/4*d_2^2*l_2*rho; % Link 2
% m3 = pi/4*d_3^2*l_3*rho; % Link 3
% m4 = 0;
% m5 = pi/4*d_4^2*l_4*rho; % Link 4
% m6 = 2; % Ball mass
% m = [m1 m2 m3 m4 m5 m6];

% %Hollow Links
% 
% % Cross-sectional areas of each arm member
% d_1 = robot.parameters.d_1;
% d_2 = robot.parameters.d_2;
% d_3 = robot.parameters.d_3;
% d_4 = robot.parameters.d_4;
% 
% %inner diameters for wall thickness of .0075
% d1i = d_1-.0075;
% d2i = d_2-.0075;
% d3i = d_3-.0075;
% d4i = d_4-.0075;
% 
% A1 = pi/4*(d_1^2-d1i^2);
% A2 = pi/4*(d_2^2-d2i^2);
% A3 = pi/4*(d_3^2-d3i^2);
% A4 = pi/4*(d_4^2-d4i^2);
% 
% %Constants for deflection,SF
% g = robot.const.g;
% rho = robot.const.rho;
% E = robot.const.E;
% Sy = robot.const.Sy;
% 
% 
% l_1 = robot.parameters.l_1;
% l_2 = robot.parameters.l_2;
% l_3 = robot.parameters.l_3;
% l_4 = robot.parameters.l_4;
% 
% m1 = A1*l_1*rho;
% m2 = A2*l_2*rho;
% m3 = A3*l_3*rho;
% m4=0;
% m5 = A4*l_4*rho;
% m6 = 2;
% m = [m1 m2 m3 m4 m5 m6];

%Hollow rectangular links

% Cross-sectional areas of each arm member
d_1 = robot.parameters.d_1;
d_2 = robot.parameters.d_2;
d_3 = robot.parameters.d_3;
d_4 = robot.parameters.d_4;

%inner diameters for wall thickness of .0075
b1 = d_1-.005;
b2 = d_2-.005;
b3 = d_3-.005;
b4 = d_4-.005;

A1 = d_1^2-b1^2;
A2 = d_2^2-b2^2;
A3 = d_3^2-b3^2;
A4 = d_4^2-b4^2;

%Constants for deflection,SF
g = robot.const.g;
rho = robot.const.rho;
E = robot.const.E;
Sy = robot.const.Sy;


l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;

m1 = A1*l_1*rho;
m2 = A2*l_2*rho;
m3 = A3*l_3*rho;
m4=0;
m5 = A4*l_4*rho;
m6 = 2;
m = [m1 m2 m3 m4 m5 m6];



% Motor masses (kg)
motor1 = 5;
motor2 = 2;
motor3 = 0;
motor4 = 0.5;
motor5 = 0.5;
motor_m = [motor1 motor2 motor3 motor4 motor5 0];

g = 9.81; % m/s^2

% Moment of inertia calculations
I_x = @(l,d) (pi/4*d^2*l*rho)/12*(3*(d/2)^2+l^2);
I_z = @(l,d) (pi/4*d^2*l*rho)/2*(d/2)^2;
I = cell(1,6);
I{1} = diag([0, 0, 0]);
I{2} = diag([I_z(l_2,d_2), I_x(l_2,d_2) + motor3*l_2^2, I_x(l_2,d_2) + motor3*l_2^2]);
I{3} = diag([I_x(l_3,d_3) + (motor4+motor5)*l_2^2, I_z(l_3,d_3), I_x(l_3,d_3) + (motor4+motor5)*l_2^2]);
I{4} = diag([0, 0, 0]);
I{5} = diag([I_z(l_4,d_4), I_x(l_4,d_4) + m6*l_4^2, I_x(l_4,d_4) + m6*l_4^2]);
I{6} = diag([0, 0, 0]);

% Get transform matrices of robot
robot = basketInit();
joint_angles = [the(1), the(2), the(3), the(4), the(5)];
[~,basket_T] = basketFK(joint_angles, robot);

% Initialize variables
d_the(6) = 0;
dd_the(6) = 0;
R = cell(1,6);
P = cell(1,6);
Pc = cell(1,6);
w = cell(1,6);
wd = cell(1,6);
vd = cell(1,6);
vdc = cell(1,6);
F = cell(1,6);
N = cell(1,6);
f = cell(1,6);
n = cell(1,6);

% Forward velocity propagation
for i = 1:6
    fprintf('Velocity propagation for link %d\n',i);
    R{i} = basket_T{i}(1:3,1:3)';
    P{i} = basket_T{i}(1:3,4);
    Pc{i} = (m(i)*(P{i}/2) + motor_m(i)*P{i})/(m(i)+motor_m(i));
    if (i == 1)
        w{1} = [0; 0; d_the(1)];
        wd{1} = [0; 0; dd_the(1)];
        vd{1} = [0; 0; g];
    else
        w{i} = R{i}*w{i-1} + [0; 0; d_the(i)];
        wd{i} = R{i}*wd{i-1} + cross(R{i}*w{i-1},[0; 0; d_the(i)]) + [0; 0; dd_the(i)];
        vd{i} = R{i}*(cross(wd{i-1},P{i}) + cross(w{i-1},cross(w{i-1},P{i})) + vd{i-1});
    end
    vdc{i} = cross(wd{i},Pc{i}) + cross(w{i},cross(w{i},Pc{i})) + vd{i};
    F{i} = (m(i)+motor_m(i))*vdc{i};
    N{i} = I{i}*wd{i} + cross(w{i},I{i}*w{i});
end

% Backwards force/moment propagation
for i = 6:-1:1
    fprintf('Calculating torques for joint %d\n',i);
    if (i == 6)
        f{6} = F{6};
        n{6} = N{6};
    else
        f{i} = R{i+1}'*f{i+1} + F{i};
        n{i} = N{i} + R{i+1}'*n{i+1} + cross(Pc{i+1},F{i}) + cross(P{i+1},R{i+1}'*f{i+1});
    end
end

% Return forces/moments at each joint
forces = zeros(3,size(f,2));
moments = zeros(3,size(n,2));
for i = 1:size(f,2)
    forces(1:3,i) = f{i};
    moments(1:3,i) = n{i};
end

end
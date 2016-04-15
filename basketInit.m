function robot = basketInit()

% Robot dimensions in millimeters
l_1 = 2.000; % [m] Robot is twice as tall as FANUC, because robot is a ...
%basketball player.
l_2 = 0.900; % [m]
l_3 = 1.600; % [m]
l_4 = 0.180; % [m]

d_1 = 0.03; % [m]
d_2 = 0.03; % [m]
d_3 = 0.03; % [m]
d_4 = 0.03; % [m]

% Physical Parameters for aluminium
%http://asm.matweb.com/search/SpecificMaterial.asp?bassnum=MA6061t6

rho = 2.7*10^3; % kg/m^3
E=68.5*10^9; %N/m2
Sut=310*10^9; %N/m2
Sy=276*10^9; %n/m2

robot.const.rho = rho;
robot.const.E=E;
robot.const.Sut=Sut;
robot.const.Sy=Sy;
robot.const.g = 9.81; % m/s^2S


robot.home_angles = [0;0;0;0;pi/2];
robot.home_pos = [l_3+l_4; 0; l_1+l_2];

% Fill in FANUC D-H parameters and other necessary parameters 
robot.parameters.l_1 = l_1;
robot.parameters.l_2 = l_2;
robot.parameters.l_3 = l_3;
robot.parameters.l_4 = l_4;

robot.parameters.d_1 = d_1;
robot.parameters.d_2 = d_2;
robot.parameters.d_3 = d_3;
robot.parameters.d_4 = d_4;

robot.parameters.m_1 = pi/4*d_1^2*l_1*rho;
robot.parameters.m_2 = pi/4*d_2^2*l_2*rho;
robot.parameters.m_3 = pi/4*d_3^2*l_3*rho;
robot.parameters.m_4 = pi/4*d_4^2*l_4*rho;

% Basket base (zero) frame relative to the "station" frame
robot.base = makehgtform('translate',[0,0,l_1]);

% Basket joint limits (deg) (range then speed?)
deg2rad = pi/180;
robot.joint_limits{1} = [-180,180]*deg2rad;
robot.joint_limits{2} = [-180,180]*deg2rad;
robot.joint_limits{3} = [-180,180]*deg2rad;
robot.joint_limits{4} = [-180,180]*deg2rad;
robot.joint_limits{5} = [-180,180]*deg2rad;

% Set bounds on the cartesian workspace of the robot for plotting in the
% form:  [ xmin, xmax, ymin, ymax, zmin, zmax]

robot.workspace = [-2.739*cos(30), 2.739, -2.739, 2.739, -1.721, 2.238];

% Set colors to be drawn for each link and associated frame
robot.colors{1} = [1,0,0];
robot.colors{2} = [0,1,0];
robot.colors{3} = [1,1,0];
robot.colors{4} = [0,0,1];
robot.colors{5} = [1,0,1];
robot.colors{6} = [0,1,1];

%Basket goal location
robot.goal.pos = [0; 1.5; 2];
robot.goal.predunking = robot.goal.pos + [0; -l_4; l_4];
%Changing the goal position will require prebasket end effector position to
%require adjustment, as it is not dynamic as of right now.

%Ball Mass
robot.ball.mass = 0; %[kg]


end
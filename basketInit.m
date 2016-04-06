function robot = basketInit()

% Robot dimensions in millimeters
l_1 = 2.000; % [m] Robot is twice as tall as FANUC, because robot is a ...
%basketball player.
l_2 = 0.900; % [m]
l_3 = 1.600; % [m]
l_4 = 0.180; % [m]

% Fill in FANUC D-H parameters and other necessary parameters 
robot.parameters.l_1 = l_1;
robot.parameters.l_2 = l_2;
robot.parameters.l_3 = l_3;
robot.parameters.l_4 = l_4;

% Basket base (zero) frame relative to the "station" frame
robot.base = makehgtform('translate',[0,0,l_1]);

% Basket joint limits (deg) (range then speed?)
deg2rad = pi/180;
robot.joint_limits{1} = [-150,150]*deg2rad;
robot.joint_limits{2} = [-80,80]*deg2rad;
robot.joint_limits{3} = [-80,80]*deg2rad;
robot.joint_limits{4} = [-240,240]*deg2rad;
robot.joint_limits{5} = [-120,120]*deg2rad;

% Set bounds on the cartesian workspace of the robot for plotting in the
% form:  [ xmin, xmax, ymin, ymax, zmin, zmax]

%robot.workspace = [-2739*cos(30), 2739, -2739, 2739, -1721, 2238];

% Set colors to be drawn for each link and associated frame
robot.colors{1} = [1,0,0];
robot.colors{2} = [0,1,0];
robot.colors{3} = [0,0,1];
robot.colors{4} = [1,1,0];

%Basket goal location
pos = [0.400; 0; 0.400]; %just a guess [m]
orient = eye(3); %just a regular basketball goal for now
robot.goal.pos_ore = [orient, pos; 0, 0, 0, 1];

%Ball Mass
robot.ball.mass = 2; %[kg]

%Physical Constants
robot.const.g = 9.81; % m/s^2

end
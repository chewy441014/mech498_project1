function [ handles ] = drawBasket( joint_angles, pos_ball, robot )
% MECH 498/598 - Intro to Robotics - Spring 2016
%
%    DESCRIPTION - Plot a graphical representation of the robot 
%    Industrial robot with attached coordinate frames
%
%    INPUTS - joint_angles is a 5-element vector of robot joint angles to
%                  specify the pose in which we wish to draw the robot.
%    
%             fanuc is a structure generated by fanucInit()
%
%    OUTPUTS - handles is a vector of graphics handles corresponding to the
%                  moving frames attached to the robot

% Create a cell array of FANUC forward kinematics transforms
[~,robot_T] = basketFK(joint_angles,robot);

% Shorten variable names
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;

% Plot scaling properties
origin_size = 20;
marker_size = 10;
vector_size = 0.05*max(abs(diff(reshape(robot.workspace,2,3))));

% Create figure window
figure('Color','w');

% Create axes object
ax = axes('XLim',robot.workspace(1:2),'YLim',robot.workspace(3:4),...
   'ZLim',robot.workspace(5:6));
vw = [31.3,22.8];
set(gca,'View',vw);
grid on;
axis equal;
xlabel('X (mm)','FontSize',16);
ylabel('Y (mm)','FontSize',16);
zlabel('Z (mm)','FontSize',16);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Draw Robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create link 0 and frame 0
h = drawRobotFrame([0,0,0]);
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
circ = linspace(0,2*pi,50);
L_0 = line(0.1*cos(circ),0.1*sin(circ),...
    -l_1*ones(length(circ)),...
    'Color','k','LineWidth',1.5);
set(L_0,'Parent',hg);
T_0 = hgtransform('Parent',ax,'Matrix',makehgtform('translate',[0,0,l_1]));
set(hg,'Parent',T_0);

% Create link 1 and frame 1
h = drawRobotFrame(robot.colors{1});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_1 = line([0,0],[0,0],[-l_1,0],...
    'Color',robot.colors{1},'LineWidth',1.5);
set(L_1,'Parent',hg);
T_1 = hgtransform('Parent',T_0,'Matrix',robot_T{1});
set(hg,'Parent',T_1);

% Create link 2 and frame 2
h = drawRobotFrame(robot.colors{2});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_2 = line([0,l_2],[0,0],[0,0],...
    'Color',robot.colors{2},'LineWidth',1.5);
set(L_2,'Parent',hg);
T_2 = hgtransform('Parent',T_1,'Matrix',robot_T{2});
set(hg,'Parent',T_2);

% Create link 3 and frame 3
h = drawRobotFrame(robot.colors{3});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_3 = line([0,0],[-l_3,0],[0,0],...
    'Color',robot.colors{3},'LineWidth',1.5);
set(L_3,'Parent',hg);
T_3 = hgtransform('Parent',T_2,'Matrix',robot_T{3});
set(hg,'Parent',T_3);

% Create link 4 and frame 4
h = drawRobotFrame(robot.colors{4});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_4 = line([0,0,0],[0,0,0],[0,0,0],...
    'Color',robot.colors{4},'LineWidth',1.5);
set(L_4,'Parent',hg);
T_4 = hgtransform('Parent',T_3,'Matrix',robot_T{4});
set(hg,'Parent',T_4);

% Create link 5 and frame 5
h = drawRobotFrame(robot.colors{5});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_5 = line([0,0],[-l_4,0],[0,0],...
    'Color',robot.colors{5},'LineWidth',1.5);
set(L_5,'Parent',hg);
T_5 = hgtransform('Parent',T_4,'Matrix',robot_T{5});
set(hg,'Parent',T_5);

% Create link 6 and frame 6
h = drawRobotFrame(robot.colors{6});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_6 = line([0,0,0],[0,0,0],[0,0,0],...
    'Color',robot.colors{6},'LineWidth',1.5);
set(L_6,'Parent',hg);
T_6 = hgtransform('Parent',T_5,'Matrix',robot_T{6});
set(hg,'Parent',T_6);

% Create Ball
ball = hggroup('Parent',ax);
O = line(pos_ball(1), pos_ball(2), pos_ball(3),'Color',[0,0,0],'Marker','.','MarkerSize',50);
set(O,'Parent',ball);

% Create Basket
scale = 0.15;
height = 2;
loc = robot.goal.pos;
goal = hggroup('Parent',ax);
t = linspace(0,2*pi,200);
x = scale*cos(t);
y = scale*sin(t);
p1 = line(x,y,zeros(length(t)),'Color','red','LineWidth',5); % Rim
set(p1,'Parent',goal);
% p2 = line(0.5*x, 0.5*y, scale*-height*ones(length(t))); % Base
% set(p2,'Parent',goal);
t = linspace(0,pi/4,100);
for i = 0:15
    x = scale*cos(t + i*pi/8).*(-t*2/pi + 1);
    y = scale*sin(t + i*pi/8).*(-t*2/pi + 1);
    z = scale*-t*4*(height)/pi;
    p3 = line(x, y, z, 'Color','k','LineWidth',2);
    set(p3,'Parent',goal);
end
t = linspace(0,pi/4,100);
for i = 0:15
    x = scale*sin(t + i*pi/8).*(-t*2/pi + 1);
    y = scale*cos(t + i*pi/8).*(-t*2/pi + 1);
    z = scale*-t*4*(height)/pi;
    p4 = line(x, y, z,'Color', 'k','LineWidth',2);
    set(p4,'Parent',goal);
end
T_goal = makehgtform('translate',loc);
T = hgtransform('Parent',ax,'Matrix',T_goal);
set(goal,'Parent',T);

% Render graphics
set(gcf,'Renderer','openGL');
drawnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Return hgtransform handles
handles = [T_1,T_2,T_3,T_4,T_5,T_6,ball];

    function h = drawRobotFrame( color )
         
        % Plot reference frame
        X_b = [vector_size,0,0,1]';
        Y_b = [0,vector_size,0,1]';
        Z_b = [0,0,vector_size,1]';
        h(1) = line(0,0,0,'Marker','.','MarkerSize',origin_size,'Color',color);
        h(2) = line([0,X_b(1)],[0,X_b(2)],[0,X_b(3)],'LineWidth',1.5,'Color',color);
        h(3) = line([0,Y_b(1)],[0,Y_b(2)],[0,Y_b(3)],'LineWidth',1.5,'Color',color);
        h(4) = line([0,Z_b(1)],[0,Z_b(2)],[0,Z_b(3)],'LineWidth',1.5,'Color',color);
        h(5) = line(X_b(1),X_b(2),X_b(3),'LineWidth',1.5,'Marker','x','MarkerSize',marker_size,'Color',color);
        h(6) = line(Y_b(1),Y_b(2),Y_b(3),'LineWidth',1.5,'Marker','o','MarkerSize',marker_size,'Color',color);
        h(7) = line(Z_b(1),Z_b(2),Z_b(3),'LineWidth',1.5,'Marker','d','MarkerSize',marker_size,'Color',color);
    end


end




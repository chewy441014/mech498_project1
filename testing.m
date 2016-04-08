robot = basketInit();
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;
theta_x = pi/2;
R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
T = R_x;
% T(1:4,4) = [l_3 0 l_2+l_4 1]';
T(1:4,4) = [2 0 2 1]';
[~, joint_angles] = basketIK(T,zeros(5,1),robot);
drawBasket(joint_angles,robot)

%%
robot = basketInit();
pos_ball = [5 0 0];
vel_ball = [-2 0 10];
dt = 0.01;
[~, ball_trajectory] = ballTrajectory(pos_ball, vel_ball, robot, dt);
ball_trajectory
robot.handles = drawBasket([0 0 0 0 0],robot);
F(size(ball_trajectory,2)) = struct('cdata',[],'colormap',[]);
% robotVideo = VideoWriter('robotVideo.avi');
% open(robotVideo);
for t = 1:size(ball_trajectory,2)
    setBasket([0 0 0 0 0],robot);
    O = robot.handles(7).Children;
    set(O, 'XData', ball_trajectory(1,t));
    set(O, 'YData', ball_trajectory(2,t));
    set(O, 'ZData', ball_trajectory(3,t));
%     writeVideo(robotVideo, getframe)
end
% close(robotVideo);

%%
pos_ball = [4 0 0];
vel_ball = [-2 0 10];
simulateBasket(pos_ball, vel_ball);
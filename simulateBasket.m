function simulateBasket(pos_ball, vel_ball)
dt = 0.01;
% vel_2_intersect=2; %set velocity for robot to move to intersection point
%used for determining t_f

robot = basketInit();
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;

[~,ball_traj] = ballTrajectory(pos_ball, vel_ball, robot, dt);
home_pos = [l_3+l_4; 0; l_1+l_2];

smallest_dist = 99999999;
intersect_dt = 0;
for i = 1:size(ball_traj,2)
    dist = norm(home_pos - ball_traj(:,i));
    if(dist < smallest_dist)
        smallest_dist = dist;
        intersect_dt = i;
        intersect_time = i*dt;
    end
end
tangent = ball_traj(:,intersect_dt) - ball_traj(:,intersect_dt-1);
theta_y = tan(tangent(1)/tangent(3));
theta_z = tan(tangent(2)/tangent(1));
Ry = [cos(theta_y) 0 sin(theta_y);
    0 1 0;
    -sin(theta_y) 0 cos(theta_y)];
Rz = [cos(theta_z) -sin(theta_z) 0;
    sin(theta_z) cos(theta_z) 0;
    0 0 1];
R = Rz*Ry;
T = R;
T(1:4,4) = [ball_traj(:,intersect_dt); 1];
[~, desired_joint_angles] = basketIK(T,[0 0 0 0 0],robot);
drawBasket(desired_joint_angles, robot);
hold on
plot3(ball_traj(1,:),ball_traj(2,:),ball_traj(3,:),'.:');


% %Find intersection point and time when the ball arrives at the
% %intersection point
% home_angles = robot.home_angles;
% home_position = robot.home_position;
% g=robot.const.g;
% vxb=vel_ball(1);
% vyb=vel_ball(2);
% vzb=vel_ball(3);
% totaltime=2*vzb/g;
% syms t
% A=(vxb*t)^2+(vyb*t)^2+(vzb*t-0.5*g*t^2-home_position(3))^2;
% B=2*vxb^2+2*vyb^2+2*(vzb*t-0.5*g*t^2-home_position(3))*(vzb-g*t);
% dt=0.5*A^(-1/2)*B;
% d2t=-B^2/4*A^(-3/2)+0.5*A^(-1/2)*(2*(vzb-g*t)^2-2*g*(vzb*t-0.5*g*t^2-home_position(3)));
% t_intersect=solve(0.5*A^(-1/2)*B==0,t);
% tsol=[];
% if length(t_intersect)!=0
%     
%     for i=1:length(t_intersect)
%         if isreal(t_intersect(i))
%             if subs(d2t,t_intersect(i))>0
%                 tsol=[tsol t_intersect(i)];
%             end
%         end
%                 
%     end
% else
%     if home_position(3)<sqrt((vxb*totaltime)^2+(vyb*totaltime)^2+(vzb...
%         *totaltime-0.5*g*totaltime^2-home_position(3))^2)
%         tsol=0;
%     else
%         tsol=totaltime;
%     end
% end
% tsol=min(tsol);
% 
% 
% intersection = ball_trajectory(:,round(tsol/dt));
% t_intersect=tsol;
% 
% vel_intersection=[vel_ball(1),vel_ball(2),vel_ball(3)-g*t_intersect];
% vx=vel_intersection(1);
% vy=vel_intersection(2);
% vz=vel_intersection(3);
% ball_angles=[atan2(vy,vz);atan2(vz,vx);atan2(vx,vy)];
% roll=ball_angles(1)+pi;
% yaw=ball_angles(2)+pi;
% pitch=ball_angels(3)+pi;
% 
% Rx=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
% Ry=[cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
% Rz=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
% R=Rz*Ry*Rx;
% Tball=zeros(4);
% Tball(1:3,1:3)=R;
% Tball(4,4)=1;
% Tball(1:3,4)=intersection;
% 
% joint_angles_intersection = basketIK(Tball, home_angles, robot);

t_f=ceil((intersection-home_position)/vel_2_intersect);
i=0:dt:t_f;

%Moving to the Ball Intersection Location - PID control
prev_joint_angles=home_angles;
[is_sol, joint_angles] = basketIK(position, prev_joint_angles, robot);
for i = 1:dt:t_f
    %Theta_ref is the intersection point
    %Theta_init is the zero position of the robot
    joint_angles_mat = controlBasketPID(theta_init, theta_ref, t_f, robot);
end

%Catching the Ball and Remaining Stationary (Impulse Input)

%Moving to the Pre-Basket Position

%Control Law for Dunking

%Draw the Robot


end
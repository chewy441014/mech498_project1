function simulateBasket(pos_ball, vel_ball)
dt = 0.01;
vel_2_intersect=2; %set velocity for robot to move to intersection point
%used for determining t_f

robot = basketInit();

ball_trajectory = ballTrajectory(pos_ball, vel_ball, robot, dt);

%Find intersection point and time when the ball arrives at the
%intersection point
home_angles=robot.home_angles;
home_position=robot.home_position;
g=robot.const.g;

intersection = [];

vel_intersection=[vel_ball(1),vel_ball(2),vel_ball(3)-g*t_intersection];
vx=vel_intersection(1);
vy=vel_intersection(2);
vz=vel_intersection(3);
ball_angles=[atan2(vy,vz);atan2(vz,vx);atan2(vx,vy)];
roll=ball_angles(1)+pi;
yaw=ball_angles(2)+pi;
pitch=ball_angels(3)+pi;

Rx=[1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry=[cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
Rz=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
R=Rz*Ry*Rx;
Tball=zeros(4);
Tball(1:3,1:3)=R;
Tball(4,4)=1;
Tball(1:3,4)=intersection;

joint_angles_intersection = basketIK(Tball, home_angles, robot);

t_f=ceil((intersection-home_position)/vel_2_intersect);
t=0:dt:t_f;

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
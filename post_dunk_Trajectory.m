function [ball_trajectory, timevector] = post_dunk_Trajectory(init_vel, init_pos, dt, robot)
%init_vel is a 3x1 velocity vector
%init_pos is a 3x1 velocity vector
%they are the velocity and position at which the ball is dunked
g = robot.const.g;
damping = 3; %the ratio of energy lost at each bounce
h = 0.005; %the limit of the height of the last bounce
num_bounce = 0;
vx = init_vel(1);
vy = init_vel(2);
vz = init_vel(3);
h0 = init_pos(3) + vz^2/2/g; %initial equivalent height of the ball
hi = h0;

t0 = (vz + sqrt(vz^2+2*g*init_pos(3)))/g;
t_set = [t0];
h_set = [h0];

while hi > h
    hi = hi/damping;
    num_bounce = num_bounce + 1;
    h_set(num_bounce + 1) = hi;
    t_set(num_bounce + 1) = 2*sqrt(2*hi/g) + t_set(num_bounce);
    
end

totaltime = sum(t_set);
timecell = cell(1, num_bounce);
timecell{1} = 0:dt:round(t0/dt)*dt;
pos = cell(1,num_bounce);
pos{1} = [init_pos(1) + vx*timecell{1}; init_pos(2) + vy*timecell{1}; ...
    init_pos(3) + vz*timecell{1} - 0.5*g*timecell{1}.^2];

for i = 1:num_bounce - 1
    timepoint1 = round(t_set(i)/dt)*dt;
    timepoint2 = round(t_set(i+1)/dt)*dt;
    timecell{i+1} = timepoint1:dt:timepoint2;
    v_init = sqrt(2*g*h_set(i+1));
    time = timecell{i+1} - timecell{i}(end);
    pos{i+1} = [init_pos(1) + vx*timecell{i+1}; init_pos(2) + vy*timecell{i+1}; v_init*time - 0.5*g*time.^2];
    
end

timevector = cat(2,timecell{:});
ball_trajectory = cat(2, pos{:});
% robot.handles = drawBasket(robot.home_angles, init_pos, robot)
% for i = 1:size(ball_trajectory, 2)
%     O = robot.handles(7).Children;
%     set(O, 'XData', ball_trajectory(1,i));
%     set(O, 'YData', ball_trajectory(2,i));
%     set(O, 'ZData', ball_trajectory(3,i));
%     pause(0.01)
% end
end
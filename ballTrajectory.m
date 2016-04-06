function [is_solution, ball_trajectory] = ballTrajectory(pos_ball, vel_ball, robot, dt)
    %pos_ball is a 3x1 position vector [m]
    %vel_ball is a 3x1 velocity vector [m/s]

    is_solution = true;
    %The ball must start at or above the x-y plane
    if pos_ball(3) < 0
        disp('The ball must start at or above the x-y plane')
        is_solution = false;
    elseif pos_ball(3) == 0
        disp('The ball is on the ground.')
    end
    g = robot.const.g;
    a = [0; 0; -g];
    t1 = vel_ball(3)/g;
    t_f = t1 + sqrt(2*(pos_ball(3) + vel_ball(3)*t1 - 1/2*g*t1^2)/g);
    t = 0:dt:t_f;
    pos = [pos_ball(1) + vel_ball(1)*t + 1/2*a(1)*t.^2;...
        pos_ball(2) + vel_ball(2)*t + 1/2*a(2)*t.^2;...
        pos_ball(3) + vel_ball(3)*t + 1/2*a(3)*t.^2];
    ball_trajectory = pos(:,pos(3,:) > 0);

    ball_trajectory = pos;

end
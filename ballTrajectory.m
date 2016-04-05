function [is_solution, ball_trajectory] = ballTrajectory(T_ball, vel_ball, robot, dt)
    m = robot.ball.mass;
    g = robot.const.g;
    a_x = [0;0;-m*g];
end
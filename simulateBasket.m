function simulateBasket(T_ball, vel_ball)
    dt = 0.01;
    
    robot = basketInit();
    
    ball_trajectory = ballTrajectory(T_ball, vel_ball, robot, dt);
    
    %Find intersection point and time when the ball arrives at the
    %intersection point
    intersection = [];
    joint_angles_intersection = [];
    t_f = [];
    
    %Moving to the Ball Intersection Location - PID control
    [is_sol, joint_angles] = basketIK(T, prev_joint_angles, robot);
    for i = 0:dt:t_f
        %Theta_ref is the intersection point
        %Theta_init is the zero position of the robot
        joint_angles_mat = controlBasketPID(theta_init, theta_ref, t_f, robot);
    end
    
    %Catching the Ball and Remaining Stationary (Impulse Input)
    
    %Moving to the Pre-Basket Position
    
    %Control Law for Dunking
    
    %Draw the Robot
    
    
end
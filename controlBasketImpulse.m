function [joint_angles_mat, joint_velocities_mat] = ...
    controlBasketImpulse(theta_init, Kp, Kv, robot, ball_vel_init, time)

% theta_init = 5x2 matrix, 1st column is angles, 2nd column is velocities
% Kp = 1x5 proportional gain
% Kv = 1x5 derivative gain
% time = nx1 or 1xn vector, time vector with each time step
% robot = robotInit array
% joint_angles_mat = 5xn matrix of angles at each point
% joint_velocities_mat = 5xn matrix
% ball_vel_init is the ball velocity at the point of contact :: vector 


%theta_ref is always theta_init. We want the end effector to never move.

% Define the max joint torques
tau_max = 10000; % scaler [Nm]

% Define the control variables
Kp = diag(Kp);

Kv = diag(Kv);

dt = time(2) - time(1);
n = length(time);
F = zeros(3,n);
F(:,time <= 0.1) = robot.ball.mass/dt*ball_vel_init*ones(1,length(F(1,time <= 0.1)));

X = zeros(10,n); % initialize variable to hold state vector
X_dot = zeros(10,n); % initialize variable to hold state vector derivatives

for i = 1:n
    fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
    if i == 1
        X(:,i) = [theta_init(:,1); theta_init(:,2)];
    else
        X(:,i) = X(:,i-1);
    end
    %Joint Torques
    joint_angles = X(1:5,i);
    joint_vel = X(6:10,i);
    
    [M,V,G] = basketDynamics(joint_angles, joint_vel, robot);
    
    tau = - Kp*(joint_angles - theta_init(:,1))...
        - Kv*(joint_vel - theta_init(:,2)) + G;
    J = basketJacobian(joint_angles);

%     table(joint_angles,joint_vel,tau) % For debugging
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;

    % Dynamic Model
    X_dot(1:5,i) = X(6:10,i);
    X_dot(6:10,i) = M\(tau - V - G - J'*F(:,i));

    if i > 1
        X(6:10,i) = X(6:10,i-1) + 0.5*(X_dot(6:10,i-1) ...
            + X_dot(6:10,i))*dt;
        X(1:5,i) = X(1:5,i-1) + 0.5*(X_dot(1:5,i-1) + ...
            X_dot(1:5,i))*dt;
    end
 
end
%Theta Generated for each time step
joint_angles_mat = X(1:5,:);
joint_velocities_mat = X(6:10,:);

return
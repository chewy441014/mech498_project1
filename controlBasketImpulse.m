function [joint_angles_mat, joint_velocities_mat] = ...
    controlBasketImpulse(theta_init, robot, ball_vel_init, dt)

% theta_init = 5x2 matrix, 1st column is angles, 2nd column is velocities
% theta_ref = 5x2 matrix, desired angles and velocities
% time = nx1 or 1xn vector, time vector with each time step
% robot = robotInit array
% joint_angles_mat = 5xn matrix of angles at each point
% joint_velocities_mat = 5xn matrix
% ball_vel_init is the ball velocity at the point of contact

% Define the max joint torques
tau_max = 10000; % scaler [Nm]

% Define the control variables
kp1 = 100;
kp2 = 100;
kp3 = 100;
kp4 = 100;
kp5 = 100;
Kp = diag([kp1; kp2; kp3; kp4; kp5]);

kv1 = 100;
kv2 = 100;
kv3 = 100;
kv4 = 100;
kv5 = 100;
Kv = diag([kv1; kv2; kv3; kv4; kv5]);

time = 0:dt:5;
n = length(time);
F = zeros(1,n);
F(1:5) = robot.mass.ball/2*ball_vel_init;

X = zeros(10,n); % initialize variable to hold state vector
X_dot = zeros(10,n); % initialize variable to hold state vector derivatives

for i = 1:n
    if i == 1
        X(:,i) = [theta_init(:,1); theta_init(:,2)];
    else
        %Joint Torques
        joint_angles = X(1:5,i);
        joint_vel = X(6:10,i);
        tau = - Kp*(joint_angles - theta_ref(:,1))...
            - Kv*(joint_vel - theta_ref(:,2));

        % Apply joint torque limits
        tau(tau>tau_max) = tau_max;
        tau(tau<-tau_max) = -tau_max;

        % Dynamic Model
        [M,V,G] = basketDynamics(joint_angles, joint_vel, robot);
        X_dot(1:5,i) = X(6:10,i);
        X_dot(6:10,i) = M\(tau - V - G);

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
function [joint_angles_mat, joint_velocities_mat] = ...
    controlBasketPID(theta_init, theta_ref,K_p, K_v, time, robot)

% theta_init = 5x2 matrix, 1st column is angles, 2nd column is velocities
% theta_ref = 5x2 matrix, desired angles and velocities
% time = nx1 or 1xn vector, time vector with each time step
% robot = robotInit array
% joint_angles_mat = 5xn matrix of angles at each point
% joint_velocities_mat = 5xn matrix
% K_p and K_v 5*1 vectors, converted to diagonal matrix in this file 

% Define the max joint torques
tau_max = 2000000000; % scaler [Nm]

% Define the control variables
Kp = diag(K_p);
Kv = diag(K_v);

n = length(time);
dt = time(2) - time(1);
X = zeros(10,n); % initialize variable to hold state vector
X_dot = zeros(10,n); % initialize variable to hold state vector derivatives

for i = 1:n
    fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
    if i == 1
        X(:,i) = [theta_init(:,1); theta_init(:,2)];
    else
        X(:,i) = X(:,i-1);
    end
    
    % Contorl torques
    joint_angles = X(1:5,i);
    joint_vel = X(6:10,i);
    
    % Dynamic Model
    [M,V,G] = basketDynamics(joint_angles, joint_vel, robot);
    
    tau = - Kp*(joint_angles - theta_ref(:,1))...
            - Kv*(joint_vel - theta_ref(:,2)) + G;
        
%     table(joint_angles,joint_vel,tau) % For debugging
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    X_dot(1:5,i) = X(6:10,i);
    X_dot(6:10,i) = M\(tau - V - G);
    
    % Trapexoidal Integration
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
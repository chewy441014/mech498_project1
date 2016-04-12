function [joint_angles_mat] = controlDunkPID(theta_init, trajectory, K_p, K_v, time, robot)

% theta_init = 5x2 matrix, 1st column is angles, 2nd column is velocities
% 

% Define the max joint torques
tau_max = 2000000000; % scaler [Nm]

n = length(time);
dt = time(2) - time(1);
X = zeros(10,n); % initialize variable to hold state vector
X_dot = zeros(10,n); % initialize variable to hold state vector derivatives

for i = 1:n
    if i == 1
        X(:,i) = [theta_init(:,1); theta_init(:,2)];
    else
        X(:,i) = X(:,i-1);
        % Control torques
        joint_angles = X(1:5,i);
        joint_vel = X(6:10,i);

        % Dynamic Model
        [M,V,G] = basketDynamics(joint_angles, joint_vel, robot);

        Theta_ref = trajectory(:,i);
        Theta_dot_ref = (trajectory(:,i) - trajectory(:,i-1))/dt;

        % Gravity Compensation Control
        tau = -K_p.*(joint_angles-Theta_ref)-K_v.*(joint_vel-Theta_dot_ref)+G; % control input (torque)

        % Apply joint torque limits
        tau(tau>tau_max) = tau_max;
        tau(tau<-tau_max) = -tau_max;

        X_dot(1:5,i) = X(6:10,i);
        X_dot(6:10,i) = M\(tau - V - G);

        % Trapexoidal Integration
        X(6:10,i) = X(6:10,i-1) + 0.5*(X_dot(6:10,i-1) ...
            + X_dot(6:10,i))*dt;
        X(1:5,i) = X(1:5,i-1) + 0.5*(X_dot(1:5,i-1) + ...
            X_dot(1:5,i))*dt;
    end
    
end
%Theta Generated for each time step
joint_angles_mat = X(1:5,:);

return
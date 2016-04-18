function [joint_angles_mat] = controlDunkPID(theta_init, trajectory, K_p, K_v, time, robot)

% theta_init = 5x2 matrix, 1st column is angles, 2nd column is velocities
% 

% Define the max joint torques
tau_max = 1500; % scaler [Nm]

n = length(time);
dt = time(2) - time(1);
X = zeros(10,n); % initialize variable to hold state vector
X_dot = zeros(10,n); % initialize variable to hold state vector derivatives

len = length(trajectory);
j = [];
for i = 1:len
    if i == len
        j = [j, i-1];
    else
        j = [j, i*ones(1,100)];
    end
end

for i = 1:n
    fprintf(1,'\b\b\b\b\b\b%01.4f',i/n);
    if i == 1
        X(:,i) = [theta_init(:,1); theta_init(:,2)];
    else
        X(:,i) = X(:,i-1);
        
        % Control torques
        joint_angles = X(1:5,i);
        joint_vel = X(6:10,i);

        % Dynamic Model
        [M,V,G] = basketDynamics(joint_angles, joint_vel, robot);
        
        % Trajectory interpolation
        y1 = trajectory(1:5,j(i));
        y2 = trajectory(1:5,j(i)+1);
        x1 = 100*dt*j(i);
        x2 = 100*dt*(j(i)+1);
        x = x1+dt*(mod(i,100));
        Theta_ref = y1 + (y2 - y1)*(x - x1)/(x2 - x1);
        y1 = trajectory(6:10,j(i));
        y2 = trajectory(6:10,j(i)+1);
        Theta_dot_ref = y1 + (y2 - y1)*(x - x1)/(x2 - x1);

%         table(X(1:5,i) ,X(6:10,i), Theta_ref, Theta_dot_ref)
        % Gravity Compensation Control
        tau = -K_p'.*(joint_angles-Theta_ref)-K_v'.*(joint_vel-Theta_dot_ref) + G; % control input (torque)

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
function [is_solution, joint_angles] = basketIK(T, prev_joint_angles, robot)
%BASKETIK Retrurns the joint angles for a given transform matrix
%   INPUTS: Transform matrix T describing the desitred end effector
%   position/orientation, the previous joint angles, and the fanuc structure 
%   output by fanucInit.
%
%   OUTPUTS: Outputs whether a solution exits for the given transform
%   matrix and the joint angles nearest to the previous joint angles to
%   reach the given goal.

% Set this value to true to display every configuration possible when
% fanucIK is called
showAllConfigs = false;

% Initialize joint_angles
joint_angles = zeros(5,1);

% Shorten variable names
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;
xmin = robot.workspace(1);
xmax = robot.workspace(2);
ymin = robot.workspace(3);
ymax = robot.workspace(4);
zmin = robot.workspace(5);
zmax = robot.workspace(6);

T(3,4) = T(3,4) - l_1;

% Find point (px',py',pz') given (px,py,pz). This point corresponds to the
% point at which frame 4 is located
p_4 = T*[0; 0; -l_4; 1];
px_4 = p_4(1);
py_4 = p_4(2);
pz_4 = p_4(3);

% Check that the goal point is within a cylinder where the radius, top, and
% bottom are defined by the maximum arm extension
if(sqrt(T(1,4)^2 + T(2,4)^2) > xmax || T(3,4) > zmax || T(3,4) < zmin)
    is_solution = false;
    fprintf('\n')
    disp('Target location is not within the workspace');
    pause(1)
else
    is_solution = true;
end
%These values were 2860, which is in [mm], so I used the workspace
%definitions from basketInit, since they were unused, and can be changed
%to our liking.

joint_sols = zeros(5,4);
% Rows 1-6 correspond to joints 1-6
% Column 1: Forward reach, elbow up
% Column 2: Forward reach, elbow down
% Column 3: Backward reach, elbow up
% Column 4: Backward reach, elbow down


%%%%%%%%%%%%%%%%%%%%%%%%% JOINT 1 %%%%%%%%%%%%%%%%%%%%%%%%%

% Solve for theta_1 using py' and px'
joint_sols(1, 1:2) = atan2(py_4, px_4); % Robot reaches forward
joint_sols(1, 3:4) = atan2(-py_4, -px_4); % Robot reaches backwards



%%%%%%%%%%%%%%%%%%%%%%%%% JOINTS 2 & 3 %%%%%%%%%%%%%%%%%%%%%%%%%
% Solve for joints 2 and 3 using law of cosines and px' projection into
% frame 1

% Define px' in terms of frame 1
px_4_1 = sqrt(px_4^2 + py_4^2);

% Check that distance to the goal point is geometrically reachable given
% the lengths of links 2 and 3 for FORWARD configuration
if((sqrt(px_4_1^2 + pz_4^2) < l_2 + l_3) ...
    && (l_2 < sqrt(px_4_1^2 + pz_4^2) + l_3) ...
    && (l_3 < sqrt(px_4_1^2 + pz_4^2) + l_2))

    beta_forward = atan2(pz_4, px_4_1);
    phi_forward = acos((px_4_1^2 + pz_4^2 + l_2^2 - l_3^2)...
        /(2*l_2*sqrt(px_4_1^2 + pz_4^2)));
    
    % Elbow up
    joint_sols(2,1) = -pi/2 + beta_forward + phi_forward; % theta_2 (forward, elbow up)
    
    joint_sols(3,1) = -pi/2 + acos(-(px_4_1^2 + pz_4^2 - l_2^2 - l_3^2)...
        /(2*l_2*l_3)); % theta_3 (forward, elbow up)
    
    
    % Elbow down
    joint_sols(2,2) = -pi/2 + beta_forward - phi_forward; % theta_2 (forward, elbow down)
    
    joint_sols(3,2) = -pi/2 - acos(-(px_4_1^2 + pz_4^2 - l_2^2 - l_3^2)...
        /(2*l_2*l_3)); % theta_3 (forward, elbow down)
    
else
    % If the point is not physically reachable, set the joint outside of
    % the joint limits so that the solution is thrown out
    joint_sols(2,1) = 9000*pi;
    joint_sols(2,2) = 9000*pi;
end
    
% Check that distance to the goal point is geometrically reachable given
% the lengths of links 2 and 3 for BACKWARD configuration
if(sqrt(px_4_1^2 + pz_4^2) < l_2 + l_3 ...
    && (l_2 < sqrt(px_4_1^2 + pz_4^2) + l_3) ...
    && (l_3 < sqrt(px_4_1^2 + pz_4^2) + l_2))
    
    beta_backward = atan2(pz_4, px_4_1);
    phi_backward = acos((px_4_1^2 + pz_4^2 + l_2^2 - l_3^2)...
        /(2*l_2*sqrt(px_4_1^2 + pz_4^2)));
    
    % Elbow up
    joint_sols(2,3) = pi/2 - beta_backward - phi_backward; % theta_2 (backward, elbow up)
    
    joint_sols(3,3) =  -pi/2 - acos(-(px_4_1^2 + pz_4^2 - l_2^2 - l_3^2)...
        /(2*l_2*l_3)); % theta_3 (backward, elbow up)
    
    
    % Elbow down
    joint_sols(2,4) = pi/2 - beta_backward + phi_backward; % theta_2 (backward, elbow down)
    
    joint_sols(3,4) = -pi/2 + acos(-(px_4_1^2 + pz_4^2 - l_2^2 - l_3^2)...
        /(2*l_2*l_3)); % theta_3 (backward, elbow down)
else
    % If the point is not physically reachable, set the joint outside of
    % the joint limits so that the solution is thrown out
    joint_sols(2,3) = 99999;
    joint_sols(2,4) = 99999;
end



%%%%%%%%%%%%%%%%%%%%%%%%% JOINTS 4 and 5 %%%%%%%%%%%%%%%%%%%%%%%%%

for config = 1:size(joint_sols,2)
    % Find the rotation matrix from frame 4 in the zero position to frame 6
    [~, robot_T] = basketFK(joint_sols(:,config), robot);
    T_04 = robot_T{1}*robot_T{2}*robot_T{3}*robot_T{4};
    R_04 = T_04(1:3,1:3);
    R_46 = R_04\T(1:3,1:3);

    % Solve for joint 4 and 5 using Z-Y-Z Euler angles
    joint_sols(5,config) = atan2(sqrt(R_46(3,1)^2 + R_46(3,2)^2), R_46(3,3));
    joint_sols(4,config) = atan2(R_46(2,3)/sin(joint_sols(5,config)), R_46(1,3)/sin(joint_sols(5,config)));

end


%%%%%%%%%%%%%%%%%%%%%%%%% CHECK JOINT ANGLES %%%%%%%%%%%%%%%%%%%%%%%%%

best_closeness = 999999999;
for i = 1:size(joint_sols,2)
    
    % Check whether solution is feasible
    if(~isOutsideLimits(joint_sols(:,i), robot))
        % Display all feasible configurations if desired
        if(showAllConfigs)
%             disp(i);
            drawBasket(joint_sols(:,i), robot);
        end
        
        % If robot is inside joint limits, check how close robot is to
        % previous joint angles
        closeness = getCloseness(joint_sols(:,i), prev_joint_angles);
        if(closeness < best_closeness)
            best_closeness = closeness;
            joint_angles = joint_sols(:,i);
        end
    else
%         disp(i);
    end
end
% If no solution is within the joint limits
if(best_closeness == 999999999)
    is_solution = false;
    disp('No feasible solution found');
    pause(1)
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function isOutOfBounds = isOutsideLimits(joint_angles, robot)
        % CHECKJOINTLIMITS Returns TRUE if any joint angle is out of bounds
        isOutOfBounds = false;
        for joint = 1:5
            if((joint_angles(joint) < robot.joint_limits{joint}(1)) || (joint_angles(joint) > robot.joint_limits{joint}(2)))
%                 fprintf('Joint %d is out of bounds\n',joint)
%                 fprintf('Joint: %f, Limit1: %f, Limit2: %f\n\n',joint_angles(joint),robot.joint_limits{joint}(1), robot.joint_limits{joint}(2));
                isOutOfBounds = true;
                return;
            end
        end
    end

    function closeness = getCloseness(joint_angles, prev_joint_angles)
        % CLOSENESS Returns a "closeness" rating for given joint angles
        % based on previous joint angles
        closeness = 0;
        for joint = 1:2
            % Weight joints 1 and 2 heavier due to 90 deg/sec speed
            closeness = closeness + abs(joint_angles(joint)-prev_joint_angles(joint))/90;
        end
        for joint = 3:5
            % Weight joints 3-6 lighter due to 320 deg/sec speed
            closeness = closeness + abs(joint_angles(joint)-prev_joint_angles(joint))/320;
        end
    end


end
function [is_solution, joint_angles] = basketIK(T, prev_joint_angles, robot)
%BASKETIK Retrurns the joint angles for a given transform matrix
%   INPUTS: Transform matrix T describing the desitred end effector
%   position, the previous joint angles, and the fanuc structure output by
%   fanucInit.
%
%   OUTPUTS: Outputs whether a solution exits for the given transform
%   matrix and the joint angles nearest to the previous joint angles to
%   reach the given goal.

% Set this value to true to display every configuration possible when
% fanucIK is called
showAllConfigs = false;


% Initialize joint_angles
joint_angles = zeros(6,1);

% Shorten variable names
l_1 = fanuc.parameters.l_1;
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_5 = fanuc.parameters.l_5;
l_6 = fanuc.parameters.l_6;
xmin = fanuc.workspace(1);
xmax = fanuc.workspace(2);
ymin = fanuc.workspace(3);
ymax = fanuc.workspace(4);
zmin = fanuc.workspace(5);
zmax = fanuc.workspace(6);

% Find point (px',py',pz') given (px,py,pz). This point corresponds to the
% point at which frame 4 is located
p_4 = T*[0; 0; -l_6; 1];
px_4 = p_4(1);
py_4 = p_4(2);
pz_4 = p_4(3);

% Check that the goal point is within a cylinder where the radius, top, and
% bottom are defined by the maximum arm extension
if(sqrt(T(1,4)^2 + T(2,4)^2) > 2680 || T(3,4) > 2680 || T(3,4) < -2680)
    is_solution = false;
    error('Target location is not within the workspace');
else
    is_solution = true;
end

joint_sols = zeros(6,8);
% Rows 1-6 correspond to joints 1-6
% Column 1: Forward reach, elbow up, wrist positive
% Column 2: Forward reach, elbow up, wrist negative
% Column 3: Forward reach, elbow down, wrist positive
% Column 4: Forward reach, elbow down, wrist negative
% Column 5: Backward reach, elbow up, wrist positive
% Column 6: Backward reach, elbow up, wrist negative
% Column 7: Backward reach, elbow down, wrist positive
% Column 8: Backward reach, elbow down, wrist negative


%%%%%%%%%%%%%%%%%%%%%%%%% JOINT 1 %%%%%%%%%%%%%%%%%%%%%%%%%

% Solve for theta_1 using py' and px'
joint_sols(1, 1:4) = atan2(py_4, px_4); % Robot reaches forward
joint_sols(1, 5:8) = atan2(-py_4, -px_4); % Robot reaches backwards



%%%%%%%%%%%%%%%%%%%%%%%%% JOINTS 2 & 3 %%%%%%%%%%%%%%%%%%%%%%%%%
% Solve for joints 2 and 3 using law of cosines and px' projection into
% frame 1

% Define px' in terms of frame 1
px_4_1 = sqrt(px_4^2 + py_4^2);

% Check that distance to the goal point is geometrically reachable given
% the lengths of links 2 and 3 for FORWARD configuration
if((sqrt((px_4_1 - l_2)^2 + pz_4^2) < l_3 + sqrt(l_4^2 + l_5^2)) ...
    && (l_3 < sqrt((px_4_1 - l_2)^2 + pz_4^2) + sqrt(l_4^2 + l_5^2)) ...
    && (sqrt(l_4^2 + l_5^2) < sqrt((px_4_1 - l_2)^2 + pz_4^2) + l_3))

    beta_forward = atan2(pz_4, px_4_1 - l_2);
    phi_forward = acos(((px_4_1 - l_2)^2 + pz_4^2 + l_3^2 - (l_4^2 + l_5^2))...
        /(2*l_3*sqrt((px_4_1 - l_2)^2 + pz_4^2)));
    
    % Elbow up
    joint_sols(2,1:2) = -pi/2 + beta_forward + phi_forward; % theta_2 (forward, elbow up)
    
    joint_sols(3,1:2) = -pi/2 - tan(l_4/l_5) + acos(-((px_4_1 - l_2)^2 + pz_4^2 - l_3^2 - (l_4^2 + l_5^2))...
        /(2*l_3*sqrt(l_4^2 + l_5^2))); % theta_3 (forward, elbow up)
    
    
    % Elbow down
    joint_sols(2,3:4) = -pi/2 + beta_forward - phi_forward; % theta_2 (forward, elbow down)
    
    joint_sols(3,3:4) = -pi/2 - tan(l_4/l_5) - acos(-((px_4_1 - l_2)^2 + pz_4^2 - l_3^2 - (l_4^2 + l_5^2))...
        /(2*l_3*sqrt(l_4^2 + l_5^2))); % theta_3 (forward, elbow down)
    
else
    % If the point is not physically reachable, set the joint outside of
    % the joint limits so that the solution is thrown out
    joint_sols(2,1:2) = 9000*pi;
    joint_sols(2,3:4) = 9000*pi;
end
    
% Check that distance to the goal point is geometrically reachable given
% the lengths of links 2 and 3 for BACKWARD configuration
if(sqrt((px_4_1 + l_2)^2 + pz_4^2) < l_3 + sqrt(l_4^2 + l_5^2) ...
    && (l_3 < sqrt((px_4_1 + l_2)^2 + pz_4^2) + sqrt(l_4^2 + l_5^2)) ...
    && (sqrt(l_4^2 + l_5^2) < sqrt((px_4_1 + l_2)^2 + pz_4^2) + l_3))
    
    beta_backward = atan2(pz_4, px_4_1 + l_2);
    phi_backward = acos(((px_4_1 + l_2)^2 + pz_4^2 + l_3^2 - (l_4^2 + l_5^2))...
        /(2*l_3*sqrt((px_4_1 + l_2)^2 + pz_4^2)));
    
    % Elbow up
    joint_sols(2,5:6) = pi/2 - beta_backward - phi_backward; % theta_2 (backward, elbow up)
    
    joint_sols(3,5:6) =  -pi/2 - tan(l_4/l_5) - acos(-((px_4_1 + l_2)^2 + pz_4^2 - l_3^2 - (l_4^2 + l_5^2))...
        /(2*l_3*sqrt(l_4^2 + l_5^2))); % theta_3 (backward, elbow up)
    
    
    % Elbow down
    joint_sols(2,7:8) = pi/2 - beta_backward + phi_backward; % theta_2 (backward, elbow down)
    
    joint_sols(3,7:8) = -pi/2 - tan(l_4/l_5) + acos(-((px_4_1 + l_2)^2 + pz_4^2 - l_3^2 - (l_4^2 + l_5^2))...
        /(2*l_3*sqrt(l_4^2 + l_5^2))); % theta_3 (backward, elbow down)
else
    % If the point is not physically reachable, set the joint outside of
    % the joint limits so that the solution is thrown out
    joint_sols(2,5:6) = 9000*pi;
    joint_sols(2,7:8) = 9000*pi;
end



%%%%%%%%%%%%%%%%%%%%%%%%% JOINTS 4, 5, and 6 %%%%%%%%%%%%%%%%%%%%%%%%%

for config = 1:2:size(joint_sols,2)
    % Find the rotation matrix from frame 4 in the zero position to frame 6
    [~, fanuc_T] = fanucFK(joint_sols(:,config), fanuc);
    T_04 = fanuc_T{1}*fanuc_T{2}*fanuc_T{3}*fanuc_T{4};
    R_04 = T_04(1:3,1:3);
    R_46 = R_04\T(1:3,1:3);

    % Solve for joint 4, 5, and 6 using Z-Y-Z Euler angles (positive wrist)
    joint_sols(5,config) = atan2(sqrt(R_46(3,1)^2 + R_46(3,2)^2), R_46(3,3));
    if(joint_sols(5,config) ~= 0)
        joint_sols(4,config) = atan2(R_46(2,3)/sin(joint_sols(5,config)), R_46(1,3)/sin(joint_sols(5,config)));
        joint_sols(6,config) = atan2(R_46(3,2)/sin(joint_sols(5,config)), -R_46(3,1)/sin(joint_sols(5,config)));
    else
        joint_sols(4,config) = prev_joint_angles(4);
        joint_sols(6,config) = atan2(-R_46(1,2),R_46(1,1)) - prev_joint_angles(4);
    end
    
    % Solve for joint 4, 5, and 6 using Z-Y-Z Euler angles (negative wrist)
    joint_sols(5,config+1) = atan2(-sqrt(R_46(3,1)^2 + R_46(3,2)^2), R_46(3,3));
    if(joint_sols(5,config+1) ~= 0)
        joint_sols(4,config+1) = atan2(R_46(2,3)/sin(joint_sols(5,config+1)), R_46(1,3)/sin(joint_sols(5,config+1)));
        joint_sols(6,config+1) = atan2(R_46(3,2)/sin(joint_sols(5,config+1)), -R_46(3,1)/sin(joint_sols(5,config+1)));
    else
        joint_sols(4,config+1) = prev_joint_angles(4);
        joint_sols(6,config+1) = atan2(-R_46(1,2),R_46(1,1)) - prev_joint_angles(4);
    end
end

% There are actually more than 8 possible solutions because joint 4 and
% joint 6 can rotate 360 degrees and still be in the same orientation.
% Joint 4 is not considered because it is unlikely to rotate more than 360
% degress, but joint 6 must be considered because numerous tool changes
% will result in it spinning more than 360 degrees.
for i = 1:size(joint_sols,2)
    
end



%%%%%%%%%%%%%%%%%%%%%%%%% CHECK JOINT ANGLES %%%%%%%%%%%%%%%%%%%%%%%%%

best_closeness = 999999999;
for i = 1:size(joint_sols,2)
    
    % If a solutions exists for joint 6 that is closer to the previous
    % joint 6 angle, use it (i.e. rotate by 2*pi if possible)
    if(abs((joint_sols(6,i)+2*pi) - prev_joint_angles(6)) < abs((joint_sols(6,i)) - prev_joint_angles(6))...
            && ~isOutsideLimits([0 0 0 0 0 joint_sols(6,i)+2*pi],fanuc))
        joint_sols(6,i) = joint_sols(6,i) + 2*pi;
    end
    if(abs((joint_sols(6,i)-2*pi) - prev_joint_angles(6)) < abs((joint_sols(6,i)) - prev_joint_angles(6))...
            && ~isOutsideLimits([0 0 0 0 0 joint_sols(6,i)-2*pi],fanuc))
        joint_sols(6,i) = joint_sols(6,i) - 2*pi;
    end
    
    % Check whether solution is feasible
    if(~isOutsideLimits(joint_sols(:,i), fanuc))
        % Display all feasible configurations if desired
        if(showAllConfigs)
            disp(i);
            drawFanuc(joint_sols(:,i), fanuc);
        end
        
        % If robot is inside joint limits, check how close robot is to
        % previous joint angles
        closeness = getCloseness(joint_sols(:,i), prev_joint_angles);
        if(closeness < best_closeness)
            best_closeness = closeness;
            joint_angles = joint_sols(:,i);
        end
    end
end
% If no solution is within the joint limits
if(best_closeness == 999999999)
    is_solution = false;
    error('No feasible solution found');
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function isOutOfBounds = isOutsideLimits(joint_angles, fanuc)
        % CHECKJOINTLIMITS Returns TRUE if any joint angle is out of bounds
        isOutOfBounds = false;
        for joint = 1:6
            if((joint_angles(joint) < fanuc.joint_limits{joint}(1)) || (joint_angles(joint) > fanuc.joint_limits{joint}(2)))
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
        for joint = 3:6
            % Weight joints 3-6 lighter due to 320 deg/sec speed
            closeness = closeness + abs(joint_angles(joint)-prev_joint_angles(joint))/320;
        end
    end


end
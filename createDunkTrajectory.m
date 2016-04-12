function dunk_trajectory = createDunkTrajectory(joint_angles,dt,t_f)
    if round(joint_angles(5)/(pi/2),1) ~= 1
        disp('The robots fifth joint is not in the correct position.')
        pause(1)
    end
    time = 0:dt:t_f;
    len = length(time);
    dunk_trajectory = joint_angles*ones(1,len);
    dunk_trajectory(5,:) = linspace(0,pi,len) + dunk_trajectory(5,:);
end
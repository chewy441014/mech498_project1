function dunk_trajectory = createDunkTrajectory(joint_angles,dt,t_f)
    time = 0:dt:t_f;
    len = length(time);
    dunk_trajectory = joint_angles*ones(1,len);
    dunk_trajectory(5,:) = linspace(0,pi/2,len) + dunk_trajectory(5,:);
end
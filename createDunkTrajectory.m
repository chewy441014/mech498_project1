function dunk_trajectory = createDunkTrajectory(joint_angles)
    if round(joint_angles(5)) ~= pi
        disp('The robots fifth joint is not in the correct position.')
    end
    len = 100;
    dunk_trajectory = ones(1,len)*joint_angles;
    dunk_trajectory(5,:) = linspace(0,pi,len) + dunk_trajectory(5,:);
end
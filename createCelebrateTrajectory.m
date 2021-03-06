function trajectory = createCelebrateTrajectory(joint_angles_init,dt,t_f,robot)
%%%
    time = 0:100*dt:t_f;
    len = length(time);
    [T,~] = basketFK(joint_angles_init,robot);
    T(3,4) = T(3,4) + robot.parameters.l_1;
    start_pos = T(1:3,4);
    via1 = [1,0.6,2.5]'; %change if the inverse kinematics 
    final_pos = robot.home_pos;
    home_angles = zeros(5,1);
    len2 = round(len/2);
    len3 = len - len2;
    
    path_cart_1 = via1 - start_pos;
    path_cart_1 = start_pos*ones(1,len2) + [0:path_cart_1(1)/(len2-1):path_cart_1(1); ...
        0:path_cart_1(2)/(len2-1):path_cart_1(2); ...
        0:path_cart_1(3)/(len2-1):path_cart_1(3)];
    path_cart_2 = final_pos - via1;
    path_cart_2 = via1*ones(1,len3) + [0:path_cart_2(1)/(len3-1):path_cart_2(1); ...
        0:path_cart_2(2)/(len3-1):path_cart_2(2); ...
        0:path_cart_2(3)/(len3-1):path_cart_2(3)];
    
    joint_angles_mat = zeros(5,len);
    for i = 1:len
        fprintf(1,'\b\b\b\b\b\b%01.4f',i/len);
        if i <= len2
            pos = path_cart_1(:,i);
        elseif i > len2 && i <= len
            pos = path_cart_2(:,i-len2);
        else
            disp('Something fucked up')
        end
        T1 = [1 0 0; 0 0 -1; 0 1 0]; T1(1:4,4) = [pos; 1];
        if i > 1
            [~, joint_angles] = basketIK(T1, joint_angles_mat(:,i-1), robot);
        else
            [~, joint_angles] = basketIK(T1, joint_angles_init, robot);
        end
        
        joint_angles_mat(:,i) = joint_angles';
    end
    joint_vel_mat = zeros(5,len);
    % Forward difference
    joint_vel_mat(:,1) = (joint_angles_mat(:,2) - joint_angles_mat(:,1))/(200*dt);
    % Backwards difference
    joint_vel_mat(:,end) = (joint_angles_mat(:,end) - joint_angles_mat(:,end-1))/(200*dt);
    for i = 2:len-1
        joint_vel_mat(:,i) = (joint_angles_mat(:,i+1) - ...
            joint_angles_mat(:,i-1))/(200*dt);
    end

    
    trajectory(1:5,:) = joint_angles_mat; %Joint angles
    trajectory(6:10,:) = joint_vel_mat; %Joint velocities
    n = length(joint_angles_mat);
    p = 100;
    trajectory(1:5,n+1:n+p) = home_angles*ones(1,p);
    trajectory(6:10,n+1:n+p) = zeros(5,p);
end
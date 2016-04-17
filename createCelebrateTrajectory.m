function trajectory = createCelebrateTrajectory(joint_angles,dt,t_f,robot)
%%%
    time = 0:dt:t_f;
    len = length(time);
    [T,~] = basketFK(joint_angles,robot);
    start_pos = T(1:3,4);
    via1 = [1.5,1.4,2.5]'; %change if the inverse kinematics 
    final_pos = robot.home_pos;
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
%     joint_vel_mat = zeros(5,len);
    for i = 1:len
        fprintf(1,'\b\b\b\b\b\b%01.4f',i/len);
        if i <= len2
            pos = path_cart_1(:,i);
        elseif i > len2 && i <= len
            pos = path_cart_2(:,i-len2);
        else
            disp('Something fucked up')
        end
        T1 = T(1:3,1:3); T1(1:3,4) = pos;
        if i > 1
            [~, joint_angles] = basketIK(T1, joint_angles_mat(:,i-1), robot);
        else
            [~, joint_angles] = basketIK(T1, [0 0 0 0 0], robot);
        end
        
        joint_angles_mat(:,i) = joint_angles';
%         if i > 1
%             joint_vel_mat(:,i) = (joint_angles_mat(:,i) - ...
%                 joint_angles_mat(:,i-1))/dt;
%         end
    end
    trajectory(1:5,:) = joint_angles_mat; %Joint angles
    %trajectory(6:10,:) = joint_vel_mat; %Joiint velocities
end
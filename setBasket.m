function setBasket(joint_angles, robot)
% SETBASKET Update the position of the robot after calling drawBasket

[~,robot_T] = basketFK(joint_angles,robot);
set(robot.handles(1),'Matrix',robot_T{1});
set(robot.handles(2),'Matrix',robot_T{2});
set(robot.handles(3),'Matrix',robot_T{3});
set(robot.handles(4),'Matrix',robot_T{4});
set(robot.handles(5),'Matrix',robot_T{5});
set(robot.handles(6),'Matrix',robot_T{6});
drawnow;

end


# mech498_project1
This is the github repository for mech 498 project. It is also for the mech 401 project. 

Team:
Amelia Bian
Nathan Bucki
Preston Hill
Ian Tomkinson
George Zhu

For questions:
plh1@rice.edu

List of Function

simulateBasket(T_ball, vel_ball)

robot = basketInit()

[is_solution, ball_trajectory] = ballTrajectory(T_ball, vel_ball, dt)

dunk_trajectory = createDunkTrajectory(robot)

joint_angles_mat = controlBasketPID(theta_init, theta_ref, t_f, robot)

[M, V, G] = basketDynamics(joint_angles, joint_vel)

J = basketJacobian(joint_angles, robot)

[T,robot_T] = basketFK(joint_angles,robot)

[is_solution, joint_angles] = basketIK(T, prev_joint_angles, robot)

T = dhtf(alpha, a, d, theta)

drawBasket(joint_angles)

<<<<<<< HEAD
function J = basketJacobian(joint_angles, robot)
theta1=joint_angles(1);
theta2=joint_angles(2);
theta3=joint_angles(3);
s1 = sin(joint_angles(1));
c1 = cos(joint_angles(1));
s2 = sin(joint_angles(2));
c2 = cos(joint_angles(2));
s3 = sin(joint_angles(3));
c3 = cos(joint_angles(3));


s23 = sin(joint_angles(2)+joint_angles(3));
c23 = cos(joint_angles(2)+joint_angles(3));

l2 = robot.parameters.l_2;
l3 = robot.parameters.l_3;
a3 = robot.parameters.a_3;
l4 = robot.parameters.l_4;


 %x=cos(theta1)*(l_2 + a_3*cos(theta2 + theta3) + l_4*sin(theta2 + theta3) + l_3*cos(theta2))
 %y=sin(theta1)*(l_2 + a_3*cos(theta2 + theta3) + l_4*sin(theta2 + theta3) + l_3*cos(theta2))
 z=a_3*sin(theta2 + theta3) - 1.0*l_4*cos(theta2 + theta3) + l_3*sin(theta2)

J = [-(l2 + a3*c23 + l4*s23 + l3*c2)*s1, c1*(a3*(-s23)+l4*c23-l3*s2),c1*(a3*(-s23)+l4*c23);
    (l2 + a3*c23 + l4*s23 + l3*c2)*c1, s1*(a3*(-s23)+l4*c23-l3*s2),s1*(a3*(-s23)+l4*c23);
    0, a3*c23+l4*s23+l3*c2, a3*c23+l4*s23+l3*c2];

=======
function J = basketJacobian(joint_angles, robot)

>>>>>>> origin/master
end
function J = basketJacobian(joint_angles)
theta1=joint_angles(1);
theta2=joint_angles(2);
theta3=joint_angles(3);
theta4=joint_angles(4);
theta5=joint_angles(5);

% x=0.18*cos(theta5 - 1.5708)*(1.0*cos(theta4)*(1.0*cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 1.0*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3)) + sin(theta1)*sin(theta4)) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta1)*cos(theta3)) + 0.9*cos(theta2 + 1.5708)*cos(theta1) + 1.6*cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + 1.6*sin(theta2 + 1.5708)*cos(theta1)*cos(theta3)
% y=0.9*cos(theta2 + 1.5708)*sin(theta1) - 0.18*cos(theta5 - 1.5708)*(cos(theta1)*sin(theta4) - 1.0*cos(theta4)*(cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 1.0*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3))) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta3)*sin(theta1)) + 1.6*cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) + 1.6*sin(theta2 + 1.5708)*cos(theta3)*sin(theta1)
% 
% z=0.9*sin(theta2 + 1.5708) - 1.6*cos(theta2 + theta3 + 1.5708) + 0.18*cos(theta2 + theta3 + 1.5708)*sin(theta5 - 1.5708) + 0.09*sin(theta2 + theta3 + 1.5708)*cos(theta4 - 1.0*theta5 + 1.5708) + 0.09*sin(theta2 + theta3 + 1.5708)*cos(theta4 + theta5 - 1.5708)
dxtheta1=0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta3)*sin(theta1)) + 0.18*cos(theta5 - 1.5708)*(cos(theta1)*sin(theta4) - 1.0*cos(theta4)*(cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 1.0*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3))) - 0.9*cos(theta2 + 1.5708)*sin(theta1) - 1.6*cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) - 1.6*sin(theta2 + 1.5708)*cos(theta3)*sin(theta1);

dxtheta2=1.6*cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 0.9*sin(theta2 + 1.5708)*cos(theta1) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 1.0*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3)) - 1.6*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3) - 0.18*cos(theta5 - 1.5708)*cos(theta4)*(cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta1)*cos(theta3));

dxtheta3=1.6*cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 1.0*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3)) - 1.6*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3) - 0.18*cos(theta5 - 1.5708)*cos(theta4)*(cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta1)*cos(theta3));

dxtheta4=0.18*cos(theta5 - 1.5708)*(cos(theta4)*sin(theta1) - 1.0*sin(theta4)*(cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 1.0*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3)));

dxtheta5=- 0.18*cos(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta1)*cos(theta3)) - 0.18*sin(theta5 - 1.5708)*(sin(theta1)*sin(theta4) + cos(theta4)*(cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 1.0*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3)));

dytheta1=0.18*cos(theta5 - 1.5708)*(sin(theta1)*sin(theta4) + cos(theta4)*(cos(theta2 + 1.5708)*cos(theta1)*cos(theta3) - 1.0*sin(theta2 + 1.5708)*cos(theta1)*sin(theta3))) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta1)*cos(theta3)) + 0.9*cos(theta2 + 1.5708)*cos(theta1) + 1.6*cos(theta2 + 1.5708)*cos(theta1)*sin(theta3) + 1.6*sin(theta2 + 1.5708)*cos(theta1)*cos(theta3);

dytheta2=1.6*cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 1.0*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3)) - 0.9*sin(theta2 + 1.5708)*sin(theta1) - 1.6*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3) - 0.18*cos(theta5 - 1.5708)*cos(theta4)*(cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta3)*sin(theta1));

dytheta3=1.6*cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 0.18*sin(theta5 - 1.5708)*(cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 1.0*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3)) - 1.6*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3) - 0.18*cos(theta5 - 1.5708)*cos(theta4)*(cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta3)*sin(theta1));
 
dytheta4=-0.18*cos(theta5 - 1.5708)*(cos(theta1)*cos(theta4) + sin(theta4)*(cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 1.0*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3)));

dytheta5=0.18*sin(theta5 - 1.5708)*(cos(theta1)*sin(theta4) - 1.0*cos(theta4)*(cos(theta2 + 1.5708)*cos(theta3)*sin(theta1) - 1.0*sin(theta2 + 1.5708)*sin(theta1)*sin(theta3))) - 0.18*cos(theta5 - 1.5708)*(cos(theta2 + 1.5708)*sin(theta1)*sin(theta3) + sin(theta2 + 1.5708)*cos(theta3)*sin(theta1));

dztheta1=0;

dztheta2=1.6*sin(theta2 + theta3 + 1.5708) + 0.9*cos(theta2 + 1.5708) - 0.18*sin(theta2 + theta3 + 1.5708)*sin(theta5 - 1.5708) + 0.09*cos(theta2 + theta3 + 1.5708)*cos(theta4 - 1.0*theta5 + 1.5708) + 0.09*cos(theta2 + theta3 + 1.5708)*cos(theta4 + theta5 - 1.5708);

dztheta3=1.6*sin(theta2 + theta3 + 1.5708) - 0.18*sin(theta2 + theta3 + 1.5708)*sin(theta5 - 1.5708) + 0.09*cos(theta2 + theta3 + 1.5708)*cos(theta4 - 1.0*theta5 + 1.5708) + 0.09*cos(theta2 + theta3 + 1.5708)*cos(theta4 + theta5 - 1.5708);
 
dztheta4=-0.09*sin(theta2 + theta3 + 1.5708)*(sin(theta4 - 1.0*theta5 + 1.5708) + sin(theta4 + theta5 - 1.5708));

dztheta5=0.18*cos(theta2 + theta3 + 1.5708)*cos(theta5 - 1.5708) + 0.09*sin(theta2 + theta3 + 1.5708)*sin(theta4 - 1.0*theta5 + 1.5708) - 0.09*sin(theta2 + theta3 + 1.5708)*sin(theta4 + theta5 - 1.5708);

J=[dxtheta1 dxtheta2 dxtheta3 dxtheta4 dxtheta5; dytheta1 dytheta2 dytheta3 dytheta4 dytheta5 ; dztheta1 dztheta2 dztheta3 dztheta4 dztheta5];
end
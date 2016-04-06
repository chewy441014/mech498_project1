syms d_the1 d_the2 d_the3 d_the4 d_the5 dd_the1 dd_the2 dd_the3 ...
    dd_the4 dd_the5
basket = basketInit;
joint_angles = [0, 0, 0, 0, 0];
[~,basket_T] = basketFK(joint_angles, basket);
R12 = round(basket_T{2}(1:3,1:3));
R23 = round(basket_T{3}(1:3,1:3));
R34 = round(basket_T{4}(1:3,1:3));
R45 = round(basket_T{5}(1:3,1:3));
w11 = [0; 0; d_the1];
w22 = R12*w11 + [0; 0; d_the2];
w33 = R23*w22 + [0; 0; d_the3];
w44 = R34*w33 + [0; 0; d_the4];
w55 = R45*w44 + [0; 0; d_the5];

dw11 = [0; 0; dd_the1];
dw22 = R12*dw11 + cross(R12*w11,[0; 0; d_the2]) + [0; 0; dd_the2];
dw33 = R23*dw22 + cross(R23*w22,[0; 0; d_the3]) + [0; 0; dd_the3];
dw44 = R34*dw33 + cross(R34*w22,[0; 0; d_the4]) + [0; 0; dd_the4];
dw55 = R45*dw44 + cross(R45*w22,[0; 0; d_the5]) + [0; 0; dd_the5];
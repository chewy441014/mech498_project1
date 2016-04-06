syms d_the1 d_the2 d_the3 d_the4 d_the5 dd_the1 dd_the2 dd_the3 ...
    dd_the4 dd_the5 m1 m2 m3 m4 m5

assume(d_the1,'real')
assume(d_the2,'real')
assume(d_the3,'real')
assume(d_the4,'real')
assume(d_the5,'real')
assume(dd_the1,'real')
assume(dd_the2,'real')
assume(dd_the3,'real')
assume(dd_the4,'real')
assume(dd_the5,'real')

basket = basketInit;

joint_angles = [0, 0, 0, 0, 0];
[~,basket_T] = basketFK(joint_angles, basket);
R12 = round(basket_T{2}(1:3,1:3));
R23 = round(basket_T{3}(1:3,1:3));
R34 = round(basket_T{4}(1:3,1:3));
R45 = round(basket_T{5}(1:3,1:3));
R56 = round(basket_T{6}(1:3,1:3));

P12 = round(basket_T{2}(1:3,4),2);
P23 = round(basket_T{3}(1:3,4),2);
P34 = round(basket_T{4}(1:3,4),2);
P45 = round(basket_T{5}(1:3,4),2);
P56 = round(basket_T{6}(1:3,4),2);

w11 = [0; 0; d_the1];
w22 = R12*w11 + [0; 0; d_the2];
w33 = R23*w22 + [0; 0; d_the3];
w44 = R34*w33 + [0; 0; d_the4];
w55 = R45*w44 + [0; 0; d_the5];
w66 = R56*w55;

dw11 = [0; 0; dd_the1];
dw22 = R12*dw11 + cross(R12*w11,[0; 0; d_the2]) + [0; 0; dd_the2];
dw33 = R23*dw22 + cross(R23*w22,[0; 0; d_the3]) + [0; 0; dd_the3];
dw44 = R34*dw33 + cross(R34*w22,[0; 0; d_the4]) + [0; 0; dd_the4];
dw55 = R45*dw44 + cross(R45*w22,[0; 0; d_the5]) + [0; 0; dd_the5];

dv11 = [0; 0; 0];
dv22 = [0; 0; 0];
dv33 = R23*(cross(dw22,P23) + cross(w22,cross(w22,P23))+dv22);
dv44 = R34*(cross(dw33,P34) + cross(w33,cross(w33,P34))+dv33);
dv55 = R45*(cross(dw44,P45) + cross(w44,cross(w44,P45))+dv44);

v11 = [0; 0; 0];
v22 = [0; 0; 0];
v33 = R23'*(v22 + cross(w22,P23));
v44 = R34'*(v33 + cross(w33,P34));
v55 = R45'*(v44 + cross(w44,P45));
v66 = R56'*(v55 + cross(w55,P56));

vc1 = [0; 0; 0];
vc2 = v22 + cross(w22,P23/2);
vc3 = v33 + cross(w33,P34/2);
vc4 = v55 + cross(w55,P56/2);
vc5 = v66 + cross(w66,P56);

k1 = 1/2*m1*vc1'*vc1;
k2 = 1/2*m2*vc2'*vc2;
k3 = 1/2*m3*vc3'*vc3;
k4 = 1/2*m4*vc4'*vc4;
k5 = 1/2*m5*vc5'*vc5;

k = k1 + k2 + k3 + k4 + k5; k = simplify(k); k = vpa(k,3);
pretty(k)
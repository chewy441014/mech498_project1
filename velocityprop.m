syms d_the1 d_the2 d_the3 d_the4 d_the5 dd_the1 dd_the2 dd_the3 ...
    dd_the4 dd_the5 the1 the2 the3 the4 the5

assume(the1,'real')
assume(the2,'real')
assume(the3,'real')
assume(the4,'real')
assume(the5,'real')
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

m1 = vpa(basket.parameters.m_1,3);
m2 = vpa(basket.parameters.m_2,3);
m3 = vpa(basket.parameters.m_3,3);
m4 = vpa(basket.parameters.m_4,3);
m5 = vpa(basket.ball.mass,3);

joint_angles = [the1, the2, the3, the4, the5];
[~,basket_T] = basketFK(joint_angles, basket);
R01 = vpa(basket_T{1}(1:3,1:3),3);
R12 = vpa(basket_T{2}(1:3,1:3),3);
R23 = vpa(basket_T{3}(1:3,1:3),3);
R34 = vpa(basket_T{4}(1:3,1:3),3);
R45 = vpa(basket_T{5}(1:3,1:3),3);
R56 = vpa(basket_T{6}(1:3,1:3),3);

P01 = vpa(basket_T{1}(1:3,4),3);
P12 = vpa(basket_T{2}(1:3,4),3);
P23 = vpa(basket_T{3}(1:3,4),3);
P34 = vpa(basket_T{4}(1:3,4),3);
P45 = vpa(basket_T{5}(1:3,4),3);
P56 = vpa(basket_T{6}(1:3,4),3);

w11 = [0; 0; d_the1];
w22 = R12*w11 + [0; 0; d_the2];
w33 = R23*w22 + [0; 0; d_the3];
w44 = R34*w33 + [0; 0; d_the4];
w55 = R45*w44 + [0; 0; d_the5];
w66 = R56*w55;

wd11 = [0; 0; dd_the1];
wd22 = R12*wd11 + cross(R12*w11,[0; 0; d_the2]) + [0; 0; dd_the2];
wd33 = R23*wd22 + cross(R23*w22,[0; 0; d_the3]) + [0; 0; dd_the3];
wd44 = R34*wd33 + cross(R34*w33,[0; 0; d_the4]) + [0; 0; dd_the4];
wd55 = R45*wd44 + cross(R45*w44,[0; 0; d_the5]) + [0; 0; dd_the5];
wd66 = R56*wd55 + cross(R56*w55,[0; 0; d_the6]) + [0; 0; dd_the6];

vd11 = [0; 0; 0];
vd22 = R12*(cross(wd11,P12) + cross(w11,cross(w11,P12)) + vd11);
vd33 = R23*(cross(wd22,P23) + cross(w22,cross(w22,P23)) + vd22);
vd44 = R34*(cross(wd33,P34) + cross(w33,cross(w33,P34)) + vd33);
vd55 = R45*(cross(wd44,P45) + cross(w44,cross(w44,P45)) + vd44);
vd66 = R56*(cross(wd55,P56) + cross(w55,cross(w55,P56)) + vd55);

vdc11 = [0; 0; 0];
vdc22 = cross(wd22,P23/2) + cross(w22,cross(w22,P23/2)) + vd22;
vdc33 = cross(wd33,P34/2) + cross(w33,cross(w33,P34/2)) + vd33;
vdc44 = cross(wd44,P45/2) + cross(w44,cross(w44,P45/2)) + vd44;
vdc55 = cross(wd55,P56/2) + cross(w55,cross(w55,P56/2)) + vd56;
vdc66 = cross(wd66,[0; 0; 0]) + cross(w66,cross(w66,[0; 0; 0])) + vd66;

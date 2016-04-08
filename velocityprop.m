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

m1 = vpa(basket.parameters.m_1,3); %between {0} and {1}
m2 = 0;                            %between {1} and {2}
m3 = vpa(basket.parameters.m_2,3); %between {2} and {3}
m4 = vpa(basket.parameters.m_3,3); %between {3} and {4}
m5 = 0;                            %between {4} and {5}
m6 = vpa(basket.parameters.m_4,3); %between {5} and {6}
m7 = vpa(basket.ball.mass,3);      %AT {6}

joint_angles = [the1, the2, the3, the4, the5];
[~,basket_T] = basketFK(joint_angles, basket);

R01 = vpa(basket_T{1}(1:3,1:3),3)';
R12 = vpa(basket_T{2}(1:3,1:3),3)';
R23 = vpa(basket_T{3}(1:3,1:3),3)';
R34 = vpa(basket_T{4}(1:3,1:3),3)';
R45 = vpa(basket_T{5}(1:3,1:3),3)';
R56 = vpa(basket_T{6}(1:3,1:3),3)';

P01 = vpa(basket_T{1}(1:3,4),3);
P12 = vpa(basket_T{2}(1:3,4),3);
P23 = vpa(basket_T{3}(1:3,4),3);
P34 = vpa(basket_T{4}(1:3,4),3);
P45 = vpa(basket_T{5}(1:3,4),3);
P56 = vpa(basket_T{6}(1:3,4),3);

Pc11 = P01/2;
Pc22 = P12/2;
Pc33 = P23/2;
Pc44 = P34/2;
Pc55 = P45/2;
Pc66 = P56/2;

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
wd66 = R56*wd55 + cross(R56*w55,[0; 0; 0]);

vd11 = [0; 0; 0];
vd22 = R12*(cross(wd11,P12) + cross(w11,cross(w11,P12)) + vd11);
vd33 = R23*(cross(wd22,P23) + cross(w22,cross(w22,P23)) + vd22);
vd44 = R34*(cross(wd33,P34) + cross(w33,cross(w33,P34)) + vd33);
vd55 = R45*(cross(wd44,P45) + cross(w44,cross(w44,P45)) + vd44);
vd66 = R56*(cross(wd55,P56) + cross(w55,cross(w55,P56)) + vd55);

vdc11 = [0; 0; 0];
vdc22 = cross(wd22,Pc22) + cross(w22,cross(w22,Pc22)) + vd22;
vdc33 = cross(wd33,Pc33) + cross(w33,cross(w33,Pc33)) + vd33;
vdc44 = cross(wd44,Pc44) + cross(w44,cross(w44,Pc44)) + vd44;
vdc55 = cross(wd55,Pc55) + cross(w55,cross(w55,Pc55)) + vd55;
vdc66 = cross(wd66,Pc66) + cross(w66,cross(w66,Pc66)) + vd66;

F11 = m1*vdc11;
F22 = m2*vdc22;
F33 = m3*vdc33;
F44 = m4*vdc44;
F55 = m5*vdc55;
F66 = m6*vdc66;

N11 = zeros(3,1);
N22 = zeros(3,1);
N33 = zeros(3,1);
N44 = zeros(3,1);
N55 = zeros(3,1);
N66 = zeros(3,1);

f66 = F66;
f55 = R56'*f66 + F55;
f44 = R45'*f55 + F44;
f33 = R34'*f44 + F33;
f22 = R23'*f33 + F22;
f11 = R12'*f22 + F11;

n66 = N66;
n55 = N55 + R56'*n66 + cross(Pc66,F55) + cross(P56,R56'*f66);
n44 = N44 + R45'*n55 + cross(Pc55,F44) + cross(P45,R45'*f55);
n33 = N33 + R34'*n44 + cross(Pc44,F33) + cross(P34,R34'*f44);
n22 = N22 + R23'*n33 + cross(Pc33,F22) + cross(P23,R23'*f33);
n11 = N11 + R12'*n22 + cross(Pc22,F11) + cross(P12,R12'*f22);

tau6 = [0; 0; n66];
tau5 = [0; 0; n55];
tau4 = [0; 0; n44];
tau3 = [0; 0; n33];
tau2 = [0; 0; n22];
tau1 = [0; 0; n11];

function [forces, moments] = z401ForceFinder(the, d_the, dd_the)

robot = basketInit();

m1 = 0;
m2 = robot.parameters.m_2;
m3 = robot.parameters.m_3;
m4 = 0;
m5 = robot.parameters.m_4;
m6 = robot.ball.mass;

m = [m1 m2 m3 m4 m5 m6];

g = robot.const.g;

d_the(6) = 0;
dd_the(6) = 0;

d_1 = robot.parameters.d_1;
l_1 = robot.parameters.l_1;
l_2 = robot.parameters.l_2;
l_3 = robot.parameters.l_3;
l_4 = robot.parameters.l_4;
rho = robot.const.rho;

joint_angles = [the(1), the(2), the(3), the(4), the(5)];
[~,basket_T] = basketFK(joint_angles, robot);

R = cell(1,6);
P = cell(1,6);
Pc = cell(1,6);
w = cell(1,6);
wd = cell(1,6);
vd = cell(1,6);
vdc = cell(1,6);
F = cell(1,6);
N = cell(1,6);
I = cell(1,6);
I_x = @(x) (pi/4*d_1^2*x*rho)/12*(3*((d_1/2)^2)+x^2);
I_z = @(x) (pi/4*d_1^2*x*rho)/2*(d_1/2)^2;
%If other diameters are not equal to d1 this fucks it up. 

I{1} = diag([0, 0, 0]);
I{2} = diag([I_z(l_2), I_x(l_2), I_x(l_2)]);
I{3} = diag([I_x(l_3), I_z(l_3), I_x(l_3)]);
I{4} = diag([0, 0, 0]);
I{5} = diag([I_z(l_4), I_x(l_4), I_x(l_4)]);
I{6} = diag([0, 0, 0]);

for i = 1:6
    fprintf('Velocity propagation for link %d\n',i);
    R{i} = basket_T{i}(1:3,1:3)';
    P{i} = basket_T{i}(1:3,4);
    Pc{i} = P{i}/2;
    if (i == 1)
        w{1} = [0; 0; d_the(1)];
        wd{1} = [0; 0; dd_the(1)];
        vd{1} = [0; 0; g];
    else
        w{i} = R{i}*w{i-1} + [0; 0; d_the(i)];
        wd{i} = R{i}*wd{i-1} + cross(R{i}*w{i-1},[0; 0; d_the(i)]) + [0; 0; dd_the(i)];
        vd{i} = R{i}*(cross(wd{i-1},P{i}) + cross(w{i-1},cross(w{i-1},P{i})) + vd{i-1});
    end
    vdc{i} = cross(wd{i},Pc{i}) + cross(w{i},cross(w{i},Pc{i})) + vd{i};
    F{i} = m(i)*vdc{i};
    N{i} = I{i}*wd{i} + cross(w{i},I{i}*w{i});
end

f = cell(1,6);
n = cell(1,6);
tau = sym('tau',[1,6]);
for i = 6:-1:1
    fprintf('Calculating torques for joint %d\n',i);
    if (i == 6)
        f{6} = F{6};
        n{6} = N{6};
    else
        f{i} = R{i+1}'*f{i+1} + F{i};
        n{i} = N{i} + R{i+1}'*n{i+1} + cross(Pc{i+1},F{i}) + cross(P{i+1},R{i+1}'*f{i+1});
    end
    tau(i) = n{i}(3);
    tau(i) = expand(tau(i));
end

forces = zeros(3,size(f,2));
moments = zeros(3,size(n,2));
for i = 1:size(f,2)
    forces(1:3,i) = f{i};
    moments(1:3,i) = n{i};
end

end
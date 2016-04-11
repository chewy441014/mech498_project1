the = sym('the',[1 5]);
d_the = sym('d_the',[1 6]);
dd_the = sym('dd_the',[1 6]);
m = sym('m',[1 7]);
sym g;
d_the(6) = 0;
dd_the(6) = 0;

basket = basketInit;

joint_angles = [the(1), the(2), the(3), the(4), the(5)];
[~,basket_T] = basketFK(joint_angles, basket);

R = cell(1,6);
P = cell(1,6);
Pc = cell(1,6);
w = cell(1,6);
wd = cell(1,6);
vd = cell(1,6);
vdc = cell(1,6);
F = cell(1,6);
N = cell(1,6);
for i = 1:6
    fprintf('Velocity propagation for link %d\n',i);
    R{i} = basket_T{i}(1:3,1:3)';
    P{i} = basket_T{i}(1:3,4);
    Pc{i} = P{i}/2;
    if (i == 1)
        w{1} = [0; 0; dd_the(1)];
        wd{1} = [0; 0; dd_the(1)];
        vd{1} = [0; 0; g];
    else
        w{i} = R{i}*w{i-1} + [0; 0; d_the(i)];
        wd{i} = R{i}*wd{i-1} + cross(R{i}*w{i-1},[0; 0; d_the(i)]) + [0; 0; dd_the(i)];
        vd{i} = R{i}*(cross(wd{i-1},P{i}) + cross(w{i-1},cross(w{i-1},P{i})) + vd{i-1});
    end
    vdc{i} = cross(wd{i},Pc{i}) + cross(w{i},cross(w{i},Pc{i})) + vd{i};
    F{i} = m(i)*vdc{i};
    N{i} = zeros(3,1);
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

G = sym('G',[5 1]);
V = sym('V',[5,1]);
for i = 1:5
    fprintf('Calculating G and C matrices for joint %d\n',i);
    
    C = coeffs(tau(i),g);
    G(i) = C(end);
    
    C = coeffs(tau(i),[dd_the(1),dd_the(2),dd_the(3),dd_the(4),dd_the(5),g]);
    V(i) = C(1);
end
disp('Simplifying G and V');
G = simplify(G);
V = simplify(V);

M = sym('M',[5,5]);
for i = 1:5
    fprintf('Calculating M matrix for joint %d\n',i);
    for j = 1:5
        C = coeffs(tau(i),dd_the(j));
        if(size(C,2) == 1)
            M(i,j) = 0;
        else
            M(i,j) = C(end);
        end
    end
end
disp('Simplifying M');
M = simplify(M);
% Vertical Compression
d = 0.05; % m
l = 2; % m
A = pi*(d/2)^2; % m^2
C = 0.25; % Table 4-2
E = 200e9; % N/m^2
I = pi*d^4/64; % m^4
k = sqrt(I/A);
P_cr = A*C*pi^2*E/(l/k)^2

SF = P_cr/175

%% Horizontal Deflection
F = 50; % N
M_b = 110; %Nm

l = 2; % m
d = 0.05; % m
I = pi*d^4/64; % m^4
M_b = 110; %Nm
y_max1 = F*l^3/(3*E*I);
y_max2 = M_b*l^2/(2*E*I);
y_max_total = y_max1 + y_max2
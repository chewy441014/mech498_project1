%% Given and Selected Properties

% Material Properties
% AISI 4140 Q&T (400F)
S_ut = 257; % ksi
S_y = 238; % ksi
fprintf('Use AISI 4140 Q&T (400F) (S_ut = %.0f ksi and S_y = %.0f ksi\n\n)',S_ut,S_y);

% Given Forces, Torques, and Dimensions
F1 = 2.850; % kips
F2 = 5.200; % kips
T = 14.700; % kip-in
L1 = 4.5; % in
L2 = 6.88; % in
L3 = 4.37; % in
r = 1/16; % in

%% Reaction Forces, Shear and Moment Diagrams

% Sum of moments about R1
R2 = (4.5*F1 + 11.38*F2)/15.75; % kips
R1 = F1 + F2 - R2; % kips
fprintf('Reaction at R1: %.1f lbs \n', R1*10^3);
fprintf('Reaction at R2: %.1f lbs \n', R2*10^3);

% Plot V-M Diagrams
x = [0, L1, L1+L2, L1+L2+L3];
shear = [R1, R1-F1, R1-F1-F2, R1-F1-F2+R2];
moment = [0, L1*R1, L1*R1+L2*(R1-F1), L1*R1+L2*(R1-F1)+L3*(R1-F1-F2)];
stairs(x,shear);
title('Shear Diagram');
xlabel('x (in)');
ylabel('Shear (kips)');
figure
plot(x,moment);
title('Moment Diagram');
xlabel('x (in)');
ylabel('Moment (kip-in)');

fprintf('Maximum moment occurs x = %.2f in\n\n', L1+L2);
% Moment at a given point
M = @(x) (x*moment(2)/L1)*(x<=L1) + ((x-L1)*(moment(3)-moment(2))/L2 + moment(2))*((x>L1)&(x<=L1+L2)) + ((x-L1-L2)*(-moment(3))/L3 + moment(3))*(x>L1+L2);

%% Rough, conservative first iteration
% Look at stress concentration between F1 and F2 to estimate d because it
% has a stress concentration and large moment

% Estimate using Table 7-1
K_t = 2.7;
K_ts = 2.2;

% Assume K_f = K_t, K_fs = K_ts for conservative first pass
K_f = K_t;
K_fs = K_ts;

% Marin Factors
S_e_prime = 0.5*S_ut;
ka = 2.7*S_ut^(-0.265);
kb = 0.879*1.75^(-0.107); % Guess a reasonable diameter 
kc = 1;
kd = 1;
ke = 1;
S_e = ka*kb*kc*kd*ke*S_e_prime;

% Estimate max d required for infinite life using Goodman Equation
n = 2; % Fatigue SF
x = 0.5 + 1.5 + 7; % in
M_a = M(x);
T_m = T;
d = (16*n/pi*(2*K_f*M_a/S_e + sqrt(3)*K_fs*T_m/S_ut))^(1/3);
fprintf('Moment at stress concentration between F1 and F2: %.1f lb-in \n', M_a*10^3);
fprintf('Maximum diameter to achieve fatigue SF of 2: %.2f in\n',d);

% Estimate max d required to prevent yielding
n_y = 2;
d = (16*n_y/(S_y*pi)*(2*K_t*M_a + sqrt(3)*K_ts*T_m))^(1/3);
fprintf('Maximum diameter to achieve yield SF of 2: %.2f in\n\n',d);

% Choose D1 and D2 close to the estimated diameter from previous iteration.
% Because the estimated diameter is greater than 1.75, choose two smaller
% diameters that have standard fractions.
D1 = 1.625; % in
D2 = 1.5; % in
fprintf('Choose D1 = %.4f in and D2 = %.3f in\n\n',D1,D2);

%% Detailed second iteration
% Check various points along shaft

disp('FOR STRESS CONCENTRATION BETWEEN F1 AND F2');
D = D1;
d = D2;
% Fatigue calculations at stress concentration between F1 and F2
S_e_prime = 100; % ksi, because S_ut > 200 ksi
ka = 2.7*S_ut^(-0.265);
kb = 0.879*d^(-0.107);
kc = 1;
kd = 1;
ke = 1;
S_e = ka*kb*kc*kd*ke*S_e_prime;
fprintf('Endurance Limit: %.1f ksi\n',S_e);

% Stress concentration factors between F1 and F2
K_t = 1.9; % From Figure A-15-9 using r/d = 0.042 and D/d = 1.08
K_ts = 1.35; % From Figure A-15-8 using r/d = 0.042 and D/d = 1.08

a_bending = (0.246 - 3.08e-3*S_ut + 1.51e-5*S_ut^2 - 2.67e-8*S_ut^3)^2;
K_f = 1 + (K_t - 1)/(1 + sqrt(a_bending/r));

a_torsion = (0.19 - 2.51e-3*S_ut + 1.35e-5*S_ut^2 - 2.67e-8*S_ut^3)^2;
K_fs = 1 + (K_ts - 1)/(1 + sqrt(a_torsion/r));

M_a = M(x);
T_m = T;
stress_alt = K_f*32*M_a/(pi*d^3);
stress_mid = sqrt(3)*K_fs*16*T_m/(pi*d^3);
stress_max = sqrt(stress_alt^2 + stress_mid^2);
fprintf('Alternating stress: %.2f ksi\n',stress_alt);
fprintf('Midrange stress: %.2f ksi\n',stress_mid);
fprintf('Maximum von Mises stress: %.2f ksi\n',stress_max);

n = 1/(stress_alt/S_e + stress_mid/S_ut); % Fatigue SF
n_y = S_y/stress_max; % Yielding SF
fprintf('Fatigue SF: %.2f \n',n);
fprintf('Yielding SF: %.2f\n',n_y);

f = 0.9; % Figure 6-18
a = (f*S_ut)^2/S_e;
b = -1/3*log10(f*S_ut/S_e);
stress_rev = stress_alt/(1 - stress_mid/S_ut);
N = (stress_rev/a)^(1/b); % Cycles until failure
t = N/300; % min, Time until failure
fprintf('The shaft is estimated to fail after %.0f cycles (about %.0f minutes)\n\n',N,t);

% Check the point at F2 because it is where the maximum moment occurs
disp('AT F2');
d = D2;
% Fatigue calculations
S_e_prime = 100; % ksi, because S_ut > 200 ksi
ka = 2.7*S_ut^(-0.265);
kb = 0.879*d^(-0.107);
kc = 1;
kd = 1;
ke = 1;
S_e = ka*kb*kc*kd*ke*S_e_prime;
fprintf('Endurance Limit: %.1f ksi\n',S_e);

M_a = M(L1+L2);
T_m = T;
stress_alt = 32*M_a/(pi*d^3);
stress_mid = sqrt(3)*16*T_m/(pi*d^3);
fprintf('Alternating stress: %.2f ksi\n',stress_alt);
fprintf('Midrange stress: %.2f ksi\n',stress_mid);

n = 1/(stress_alt/S_e + stress_mid/S_ut); % Fatigue SF
n_y = S_y/sqrt(stress_alt^2 + stress_mid^2); % Yielding SF
fprintf('Fatigue SF: %.2f\n',n);
fprintf('Yielding SF: %.2f\n',n_y);

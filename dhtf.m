function T = dhtf(alpha, a, d, theta)
%DHTF Returns 4x4 homogeneous transformation matrix using DH parameters
%   alpha and theta in radians

c = @(x) cos(x);
s = @(x) sin(x);

% Equation 3.6
T = [c(theta), -sin(theta), 0, a;
    s(theta)*c(alpha), c(theta)*c(alpha), -s(alpha), -s(alpha)*d;
    s(theta)*s(alpha), c(theta)*s(alpha), c(alpha), c(alpha)*d;
    0 0 0 1];

end
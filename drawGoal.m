scale = 5;

t = linspace(0,2*pi,200);
x = scale*cos(t);
y = scale*sin(t);
plot3(x,y,zeros(length(t)),'r','LineWidth',5);
hold on
% plot3(0.5*x, 0.5*y, scale*-5*ones(length(t)));

t = linspace(0,pi/4,100);
for i = 0:15
    x = scale*cos(t + i*pi/8).*(-t*2/pi + 1);
    y = scale*sin(t + i*pi/8).*(-t*2/pi + 1);
    z = scale*-t*20/pi;
    plot3(x, y, z, 'k','LineWidth',2);
end

t = linspace(0,pi/4,100);
for i = 0:15
    x = scale*sin(t + i*pi/8).*(-t*2/pi + 1);
    y = scale*cos(t + i*pi/8).*(-t*2/pi + 1);
    z = scale*-t*20/pi;
    plot3(x, y, z, 'k','LineWidth',2);
end


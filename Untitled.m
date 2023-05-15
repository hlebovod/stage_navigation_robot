close all
clearvars
clc

spiralCenter = [0; 0];

r0 = 1;
theta0 =0;

alpha = 0;
angle = [];
x = [];
y = [];
theta = 0;

while alpha < pi/3
    angle = [angle, cot(alpha)];
    alpha = alpha + 0.001;
end

while theta < 2*pi
    r = r0.*exp(angle.* (theta0 - theta));
    x  = [x, r.*cos(theta) + spiralCenter(1)];
    y =  [y, r.*sin(theta) + spiralCenter(2)];
    theta = theta + 0.05;
end

figure
hold on
grid on
plot(spiralCenter(1), spiralCenter(2), 'k+', 'LineWidth', 2)
plot(x,y, 'r')
plot(x(1), y(1), 'or')
axis equal
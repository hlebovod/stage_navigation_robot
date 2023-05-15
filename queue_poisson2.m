% repere du robot!
clear all
close all
clearvars
clc

rowlength = linspace (-2, 0, 100);
rowwidth = 4;
pas = 0.005;
d = rowwidth;
rmin =6;
D = rmin - d/2;
L = D;
alpha_circle = pi/2;
x = [];
y = [];
x1 = [];
y1 = [];

spiralCenter = [-rmin; 0];
spiralCenter1 = [rmin - d; 0];
%% First cercle
r0 = rmin;
theta = 0;
theta0 = theta;

while theta <  pi/2
    
    angle = cot(alpha_circle) .* (theta0 - theta);
    r = r0*exp(angle);
    
    x = [x, r.*cos(theta) + spiralCenter(1)];
    y = [y, r.*sin(theta) + spiralCenter(2)];
    
    theta = theta + pas;
    
end

%% Second cercle
r01 = rmin;
theta = pi/2;
theta01 = theta ;

while theta < pi
    
    angle1 = cot(alpha_circle) .* (theta01 - theta);
    r1 = r01*exp(angle1);
    
    x1 = [x1, r1.*cos(theta) + spiralCenter1(1)];
    y1 = [y1, r1.*sin(theta) + spiralCenter1(2)];
    
    theta = theta + pas;
    
end

%%
figure
hold on
grid on
plot(spiralCenter(1), spiralCenter(2), 'r+', 'LineWidth', 2)
plot(spiralCenter1(1), spiralCenter1(2), 'r+', 'LineWidth', 2)
plot(rowwidth/2, rowlength,  'black*')
plot(-rowwidth/2, rowlength,  'black*')
plot(-3*rowwidth/2, rowlength,  'black*')
plot(x,y, 'r', 'LineWidth', 2)
plot(x1,y1, 'r', 'LineWidth', 2)
plot(x(1), y(1), 'or', 'LineWidth', 2)
plot([x(length(x)) x(length(x)) + 2*rmin - d], [y(length(y)) y(length(y))], 'b', 'LineWidth', 2)
axis equal
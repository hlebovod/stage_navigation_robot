% repere du robot!
clear all
close all
clearvars
clc

rowlength = linspace (-2, 0, 100);
rowwidth = 10;
pas = 0.005;
d = rowwidth;
rmin =6;
D = rmin - d/2;
L = D;

syms alpha
eqn = 0.5*D/sin(alpha)*(exp(cot(pi - alpha)*(pi - alpha - 2*pi))) - 0.5*D*cot(alpha) - L/2 == 0;
alpha_conv2 = solve(eqn,alpha);
alpha_conv = double(alpha_conv2);
alpha_conv = round(alpha_conv*100000)/100000;

alpha_conv = mod(alpha_conv, 2*pi)
alpha_div = pi - alpha_conv
alpha_circle = pi/2;

x = [];
y = [];
x1 = [];
y1 = [];

spiralCenter = [-0.5*D; 0.5*D*cot(alpha_conv)];
spiralCenter1 = [-d+0.5*D; 0.5*D*cot(alpha_conv)];

%% First spiral
r0 = 0.5*D/sin(alpha_div);
theta = alpha_div;
theta0 = theta;

while theta <  alpha_div +pi/2
    
    angle = cot(alpha_div+pi/2) .* (theta0 - theta);
    r = r0*exp(angle);
    x = [x, r.*cos(theta) + spiralCenter(1)];
    y = [y, r.*sin(theta) + spiralCenter(2)];
    
    theta = theta + pas;
    
end

%% Second spiral
r01 = sqrt((spiralCenter(2) - y(length(y)))^2 + (spiralCenter(1) - x(length(x)))^2) ;
theta = 2*pi -alpha_conv;
theta04 = theta ;

while theta < alpha_conv - pi
    
    angle1 = cot(alpha_conv+pi/2) .* (theta04 - theta);
    r1 = r01*exp(angle1);
    
    x1 = [x1, r1.*cos(theta) + spiralCenter1(1)];
    y1 = [y1, r1.*sin(theta) + spiralCenter1(2)];
    
    theta = theta + pas;
    
end

lin = 2*abs(((abs(spiralCenter(2))) + y(length(y))) * cot(alpha_conv - pi)) + D - d;
%%
figure
hold on
grid on
plot(spiralCenter(1), spiralCenter(2), 'r+', 'LineWidth', 3)
plot(spiralCenter1(1),spiralCenter1(2), 'b+', 'LineWidth', 3)
plot(rowwidth/2, rowlength,  'black*')
plot(-rowwidth/2, rowlength,  'black*')
plot(-3*rowwidth/2, rowlength,  'black*')
plot(x,y, 'r', 'LineWidth', 2)
plot(x1,y1, 'r', 'LineWidth', 2)
plot(x(1), y(1), 'or', 'LineWidth', 2)
plot([x(length(x)) x(length(x)) + lin], [y(length(y)) y(length(y))], 'b', 'LineWidth', 2)
axis equal
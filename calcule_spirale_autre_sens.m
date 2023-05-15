% repere du robot!
clear all
close all
clearvars
clc

rowlength = linspace (-2, 0, 100);
rowwidth = 4;
pas = 0.005;
d = rowwidth;
rmin =5;
D = rmin - d/2;
L = D;

syms alpha
eqn = 0.5*D/sin(alpha)*(exp(cot(pi - alpha)*(pi - alpha - 2*pi))) - 0.5*D*cot(alpha) - L/2 == 0;
alpha_conv2 = solve(eqn,alpha);
alpha_conv = double(alpha_conv2);
alpha_conv = round(alpha_conv*100000)/100000;

alpha_conv = mod(alpha_conv, 2*pi);
alpha_div = pi - alpha_conv;
alpha_circle = pi/2;

x = [];
y = [];
x1 = [];
y1 = [];
x2 = [];
y2 = [];
x3 = [];
y3 = [];
x4 = [];
y4 = [];

spiralCenter = [-0.5*D; 0.5*D*cot(alpha_conv)];
spiralCenter1 = [-0.5*D; abs(0.5*D*cot(alpha_conv)) + 2*L];
spiralCenter2 =  [d/2; 2*L];
spiralCenter3 = [d+0.5*D; abs(0.5*D*cot(alpha_conv)) + 2*L];
spiralCenter4 = [d+0.5*D; 0.5*D*cot(alpha_conv)];

%% First spiral
r0 = 0.5*D/sin(alpha_conv);
theta = alpha_div;
theta0 = theta;

while theta < -pi/2;
    
    angle = cot(alpha_div-pi/2) .* (theta0 - theta);
    r = r0*exp(angle);
    
    x = [x, r.*cos(theta) + spiralCenter(1)];
    y = [y, r.*sin(theta) + spiralCenter(2)];
    
    theta = theta + pas;
    
end

%% Second spiral
r01 = sqrt((spiralCenter1(2) - y(length(y)))^2 + (spiralCenter1(1) - x(length(x)))^2) ;
theta01 = pi/2 + abs(abs(-3*pi/2 - theta));
theta = theta01;

while theta > alpha_conv - pi/2 
    
    angle1 = cot(alpha_conv) .* (theta01 - theta);
    r1 = r01*exp(angle1);
    
    x1 = [x1, r1.*cos(theta) + spiralCenter1(1)];
    y1 = [y1, r1.*sin(theta) + spiralCenter1(2)];
    
    theta = theta - pas;
    thetaold = theta;
    
end

%% Third spiral / circle
r02 = sqrt((spiralCenter2(2) - y1(length(y1)))^2 + (spiralCenter2(1) - x1(length(x1)))^2);
theta02 = pi + pas;
theta = theta02;

while theta > 0
    
    angle2 = cot(alpha_circle) .* (theta02 - theta);
    r2 = r02*exp(angle2);
    
    x2 = [x2, r2.*cos(theta) + spiralCenter2(1)];
    y2 = [y2, r2.*sin(theta) + spiralCenter2(2)];
    
    theta = theta - pas;
    
end

%% Fourth spiral
r03 =  sqrt((spiralCenter3(2) - y2(length(y2)))^2 + (spiralCenter3(1) - x2(length(x2)))^2);
theta = abs(thetaold)-pas -3*pi/2;
theta03 = theta;

while theta > -pi/2;
    angle = cot(alpha_div) .* (theta03 - theta);
    r3 = r03*exp(angle);
    
    x3 = [x3, r3.*cos(theta) + spiralCenter3(1)];
    y3 = [y3, r3.*sin(theta) + spiralCenter3(2)];
    
    theta = theta - pas;
    
end

%% Fifth spiral
r04 = sqrt((spiralCenter4(2) - y3(length(y3)))^2 + (spiralCenter4(1) - x3(length(x3)))^2) ;
theta = pi/2;
theta04 = theta ;

while theta < 3*pi/2 + alpha_div
    
    angle4 = cot(alpha_conv - pi/2) .* (theta04 - theta);
    r4 = r04*exp(angle4);
    
    x4 = [x4, r4.*cos(theta) + spiralCenter4(1)];
    y4 = [y4, r4.*sin(theta) + spiralCenter4(2)];
    
    theta = theta + pas;
    
end

%%
figure
hold on
grid on
plot(spiralCenter(1), spiralCenter(2), 'r+', 'LineWidth', 2)
plot(spiralCenter1(1), spiralCenter1(2), 'b+', 'LineWidth', 2)
plot(spiralCenter2(1),spiralCenter2(2), 'g+', 'LineWidth', 2)
plot(spiralCenter3(1),spiralCenter3(2), 'b+', 'LineWidth', 2)
plot(spiralCenter4(1),spiralCenter4(2), 'r+', 'LineWidth', 2)
plot(-rowwidth/2, rowlength,  'black*')
plot(rowwidth/2, rowlength,  'black*')
plot(3*rowwidth/2, rowlength,  'black*')
plot(x,y, 'r', 'LineWidth', 2)
plot(x1,y1, 'b', 'LineWidth', 2)
plot(x2,y2, 'g', 'LineWidth', 2)
plot(x3,y3, 'b', 'LineWidth', 2)
plot(x4,y4, 'r', 'LineWidth', 2)
plot(x(1), y(1), 'or', 'LineWidth', 2)
% plot(x1(1), y1(1), 'ob', 'LineWidth', 2)
% plot(x2(1), y2(1), 'og', 'LineWidth', 2)
% plot(x3(1), y3(1), 'ob', 'LineWidth', 2)
% plot(x4(1), y4(1), 'or', 'LineWidth', 2)
axis equal
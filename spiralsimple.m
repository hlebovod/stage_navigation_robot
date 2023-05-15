close all
clearvars
clc

spiralCenter = [0; 0];

theta0 = 0;
r0 = 2;
alpha = 0.35*pi;
theta = theta0;

x = [];
y = [];
state = [5; 0; 3*pi/2];% Initial robot state
state30 = state(3,1);
stateSaved = [];
distanceInit = sqrt((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2);

while theta < 2*pi;
        
    angle = cot(alpha) .* (theta0 - theta);
    r = r0*exp(angle);
    
    x = [x, r.*cos(theta) + spiralCenter(1)];
    y = [y, r.*sin(theta) + spiralCenter(2)];
    
    theta = theta + 0.01;
end

r02 = state(1) - spiralCenter(1);

while state(3) < 2*pi
   angle2 = cot(alpha) .* (state30 - state(3));
   r2 = r02*exp(angle2);
   state1 = [state(1), r2.*cos(state(3,:)) + spiralCenter(1)];
   state2 = [state(2), r2.*sin(state(3,:)) + spiralCenter(2)];
   state3 = state(3) + 0.01;
end

figure
hold on
grid on
plot(spiralCenter(1), spiralCenter(2), 'r+', 'LineWidth', 2)
plot(state(1,:),state(2,:), 'r', 'LineWidth', 2)
plot(x,y, 'b', 'LineWidth', 2)
axis equal


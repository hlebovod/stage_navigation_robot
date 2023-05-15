close all
clearvars
clc

spiralCenter = [1.4; 0];

theta0 = 0.2;
r0 = 5;
alpha1 = 0.35*pi;
theta = 0.2;

reset = 1;

x = [];
y = [];

while theta < 2*pi - 0.2;
    
    if theta > alpha1 && reset == 1 % moment d'arrivee de spirale a sa tangente parallele a l'abscisse
        reset = 2;
        r0 =  y(length(y)) - spiralCenter(2); % le rayon du cercle
        lastr = r;
        alpha1 = pi/2; % l'angle pour tracer le cercle
        theta = pi/2;  % a partir d'ici on commence a tracer le cercle, on reinitialise le theta
        spiralCenter(1) = x(length(x)); % le centre du cercle
        
        spiralCentern(1) = x(length(x)); % pour tracer le centre du cercle
    end
    
    if theta > 3*pi/2 && reset == 2 % la fin du cercle
        reset = 3;
        alpha1 = pi - 0.35*pi;
        spiralCenter(1) = 1.4 ; % le meme centre qu'au debut pour la spirale regressive
        dis = abs(spiralCenter(1) - spiralCentern(1)); % distance entre 2 centres
        theta = -atan(r/dis) + 2*pi; % reinitialisation de theta pour le debut de la spirale regressive
        r0 = lastr/(exp( cot(alpha1)*(theta0 - theta))); % on cherche r0 tel que r de spirale devient r0 initiale
    end
    
    angle = cot(alpha1) .* (theta0 - theta);
    r = r0*exp(angle);
    
    x = [x, r.*cos(theta) + spiralCenter(1)];
    y = [y, r.*sin(theta) + spiralCenter(2)];
    
    theta = theta + 0.01;
end

rowlength = linspace (0, 20, 100);
rowwidth = 3;

figure
hold on
grid on
plot(spiralCenter(1), spiralCenter(2), 'r+', 'LineWidth', 2)
plot(spiralCentern(1),0, 'k*', 'LineWidth', 2)
% plot(rowlength, rowwidth, 'g+')
% plot(rowlength, -rowwidth, 'g+')
% plot(rowlength, rowwidth - rowwidth, 'g+')
plot(x,y, 'r', 'LineWidth', 2)
plot(x(1), y(1), 'or', 'LineWidth', 2)
axis equal


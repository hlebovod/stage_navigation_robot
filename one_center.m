clear all
close all
clearvars
clc
Ts = 0.1;
rowlength = linspace (-8, 0, 100);
rowwidth = 8;
pas = 0.005;
d = rowwidth;
rmin =20;
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

robotState = [-2*L; -d/2; 0];
stateSaved = robotState;


%% Five desired centers
spiralCenter1 = [0.5*D*cot(alpha_conv) - 2*L; -0.5*D - d/2];
spiralCenter2 = [abs(0.5*D*cot(alpha_conv)); -0.5*D - d/2];
spiralCenter3 = [abs(0.5*D*cot(alpha_conv)); d/2 + 0.5*D];
spiralCenter4 = [0.5*D*cot(alpha_conv) - 2*L; d/2 + 0.5*D];

Center =  [0;0];
Center_bis = [-L; 0];

Hwc1 = [1 0 spiralCenter1(1);
    0 1 spiralCenter1(2);
    0 0 1];

% Control inputs
control = [0.2; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Servoing parameters
lambda = 0.2;
vB = 0.2;
alphaB =alpha_div;

% Init parameters
distanceInit = sqrt((robotState(1)-spiralCenter1(1))^2 + (robotState(2)-spiralCenter1(2))^2);
distanceRef = distanceInit;
betaS = atan2(robotState(2)-spiralCenter1(2), robotState(1)-spiralCenter1(1));

alphaMax = alphaB;

initialGap = abs(sqrt((robotState(1)-spiralCenter1(1))^2 + (robotState(2)-spiralCenter1(2))^2) - distanceRef);
reo = 1; % reorientation: phase 1 then phase 2

%% Transformation matrices
beta = atan(robotState(2)/robotState(1));
while robotState(1) < -L
    Hwr = [ cos(robotState(3)) -sin(robotState(3)) robotState(1);
        sin(robotState(3)) cos(robotState(3)) robotState(2);
        0 0 1];
    
    spiralCenter_R = inv(Hwr)*[spiralCenter1;1];
    
    alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1));
    
    distance = sqrt(spiralCenter_R(1).^2 +spiralCenter_R(2).^2).^(1/2);
    distanceRef = distance;
    
    distance_d = - control(1)*cos(alpha);
    distanceRef_d = - control(1)*cos(alphaB);
    
    distance_k1 = distance + distance_d*Ts;
    
    epsilon = (-distance + distanceRef);
    
    if(abs(epsilon) > 1)
        epsilon = sign(epsilon);
    end;
    
    epsilon_d = (distance_d - distanceRef_d);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller Original
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    error = alpha - alphaB - alphaMax*epsilon;
    
    if (abs(alpha - alphaB) > 0.5*pi/180) && (reo)
          control(2) = lambda*(alpha - alphaB) + control(1)/distance*sin(alpha);
    else
        if(reo == 1)
            reo = 0;
            lambda = 0.02;
            initialGap = abs(sqrt((robotState(1)-spiralCenter1(1))^2 + (robotState(2)-spiralCenter1(2))^2) - distanceRef);
        end
          control(2) = lambda*error + control(1)/distance*sin(alpha) + alphaMax*epsilon_d;
    end;
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update robotState
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(control(2)) < 0.0001)
        robotState(1) = robotState(1) + Ts*control(1)*cos(robotState(3));
        robotState(2) = robotState(2) + Ts*control(1)*sin(robotState(3));
    else
        robotState(1) = robotState(1) + control(1)/control(2)*(sin(control(2)*Ts + robotState(3)) - sin(robotState(3)));
        robotState(2) = robotState(2) - control(1)/control(2)*(cos(control(2)*Ts + robotState(3)) - cos(robotState(3)));
    end
    robotState(3) = robotState(3) + control(2)*Ts;
    
    stateSaved = [stateSaved robotState];
end
%% Second spiral
alphaB =  alpha_conv - 2*pi;
robotState1 = [stateSaved(1,length(stateSaved));
                      stateSaved(2,length(stateSaved));
                      stateSaved(3,length(stateSaved))];
stateSaved1 = robotState1;
beta = atan2(robotState1(2),robotState1(1));

while robotState1(1) < 0
    Hwr = [ cos(robotState1(3)) -sin(robotState1(3)) robotState1(1);
        sin(robotState1(3)) cos(robotState1(3)) robotState1(2);
        0 0 1];
    
    spiralCenter_R = inv(Hwr)*[spiralCenter2;1];
    
    alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1)) - pi/2;
    
    distance = sqrt(spiralCenter_R(1).^2 +spiralCenter_R(2).^2).^(1/2);
    distanceRef = distance;
    
    distance_d = - control(1)*cos(alpha);
    distanceRef_d = - control(1)*cos(alphaB);
    
    distance_k1 = distance + distance_d*Ts;
    
    epsilon = (-distance + distanceRef);
    
    if(abs(epsilon) > 1)
        epsilon = sign(epsilon);
    end;
    
    epsilon_d = (distance_d - distanceRef_d);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller Original
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    error = alpha - alphaB - alphaMax*epsilon;
    
    if (abs(alpha - alphaB) > 0.5*pi/180) && (reo)
          control(2) = lambda*(alpha - alphaB) + control(1)/distance*sin(alpha);
    else
        if(reo == 1)
            reo = 0;
            lambda = 0.02;
            initialGap = abs(sqrt((robotState1(1)-spiralCenter1(1))^2 + (robotState1(2)-spiralCenter1(2))^2) - distanceRef);
        end
          control(2) = lambda*error + control(1)/distance*sin(alpha) + alphaMax*epsilon_d;
    end;
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update robotState
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(control(2)) < 0.0001)
        robotState1(1) = robotState1(1) + Ts*control(1)*cos(robotState1(3));
        robotState1(2) = robotState1(2) + Ts*control(1)*sin(robotState1(3));
    else
        robotState1(1) = robotState1(1) + control(1)/control(2)*(sin(control(2)*Ts + robotState1(3)) - sin(robotState1(3)));
        robotState1(2) = robotState1(2) - control(1)/control(2)*(cos(control(2)*Ts + robotState1(3)) - cos(robotState1(3)));
    end
    robotState1(3) = robotState1(3) + control(2)*Ts;
    
    stateSaved1 = [stateSaved1 robotState1];
end
%% Circle
alphaB =  pi/2;
robotState2 = [stateSaved1(1,length(stateSaved1)); stateSaved1(2,length(stateSaved1)); stateSaved1(3,length(stateSaved1))];
stateSaved2 = robotState2;
beta = atan2(robotState2(2),robotState2(1));

while robotState2(1) > 0
    Hwr = [ cos(robotState2(3)) -sin(robotState2(3)) robotState2(1);
        sin(robotState2(3)) cos(robotState2(3)) robotState2(2);
        0 0 1];
    
    spiralCenter_R = inv(Hwr)*[Center;1];
    
    alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1))  + pi;
    
    distance = sqrt(spiralCenter_R(1).^2 +spiralCenter_R(2).^2).^(1/2);
    distanceRef = distance;
    
    distance_d = - control(1)*cos(alpha);
    distanceRef_d = - control(1)*cos(alphaB);
    
    distance_k1 = distance + distance_d*Ts;
    
    epsilon = (-distance + distanceRef);
    
    if(abs(epsilon) > 1)
        epsilon = sign(epsilon);
    end;
    
    epsilon_d = (distance_d - distanceRef_d);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller Original
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    error = alpha - alphaB - alphaMax*epsilon;
    
    if (abs(alpha - alphaB) > 0.5*pi/180) && (reo)
          control(2) = lambda*(alpha - alphaB) + control(1)/distance*sin(alpha);
    else
        if(reo == 1)
            reo = 0;
            lambda = 0.02;
            initialGap = abs(sqrt((robotState1(1)-spiralCenter1(1))^2 + (robotState1(2)-spiralCenter1(2))^2) - distanceRef);
        end
          control(2) = lambda*error + control(1)/distance*sin(alpha) + alphaMax*epsilon_d;
    end;
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update robotState
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(control(2)) < 0.0001)
        robotState2(1) = robotState2(1) + Ts*control(1)*cos(robotState2(3));
        robotState2(2) = robotState2(2) + Ts*control(1)*sin(robotState2(3));
    else
        robotState2(1) = robotState2(1) + control(1)/control(2)*(sin(control(2)*Ts + robotState2(3)) - sin(robotState2(3)));
        robotState2(2) = robotState2(2) - control(1)/control(2)*(cos(control(2)*Ts + robotState2(3)) - cos(robotState2(3)));
    end
    robotState2(3) = robotState2(3) + control(2)*Ts;
    
    stateSaved2 = [stateSaved2 robotState2];
end
%% Fourth spiral
alphaB =  -alpha_div -2*pi;
robotState3 = [stateSaved2(1,length(stateSaved2)); stateSaved2(2,length(stateSaved2)); stateSaved2(3,length(stateSaved2))];
stateSaved3 = robotState3;
beta = atan2(robotState3(2),robotState3(1));

  Hwr = [ cos(robotState3(3)) -sin(robotState3(3)) robotState3(1);
        sin(robotState3(3)) cos(robotState3(3)) robotState3(2);
        0 0 1];
    
    spiralCenter_R = inv(Hwr)*[spiralCenter3;1];
    
    alphaB =  atan2(spiralCenter_R(2), spiralCenter_R(1)) + pi

while robotState3(1) > -L;
    Hwr = [ cos(robotState3(3)) -sin(robotState3(3)) robotState3(1);
        sin(robotState3(3)) cos(robotState3(3)) robotState3(2);
        0 0 1];
    
    spiralCenter_R = inv(Hwr)*[spiralCenter3;1];
    
    alpha =  -atan2(spiralCenter_R(2), spiralCenter_R(1)) + pi;
    
    distance = sqrt(spiralCenter_R(1).^2 +spiralCenter_R(2).^2).^(1/2);
    distanceRef = distance;
    
    distance_d = - control(1)*cos(alpha);
    distanceRef_d = - control(1)*cos(alphaB);
    
    distance_k1 = distance + distance_d*Ts;
    
    epsilon = (-distance + distanceRef);
    
    if(abs(epsilon) > 1)
        epsilon = sign(epsilon);
    end;
    
    epsilon_d = (distance_d - distanceRef_d);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller Original
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    error = alpha - alphaB - alphaMax*epsilon;
    
    if (abs(alpha - alphaB) > 0.5*pi/180) && (reo)
          control(2) = lambda*(alpha - alphaB) + control(1)/distance*sin(alpha);
    else
        if(reo == 1)
            reo = 0;
            lambda = 0.02;
            %initialGap = abs(sqrt((robotState1(1)-spiralCenter1(1))^2 + (robotState1(2)-spiralCenter1(2))^2) - distanceRef);
        end
          control(2) = lambda*error + control(1)/distance*sin(alpha ) + alphaMax*epsilon_d;
    end;
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update robotState
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(control(2)) < 0.001)
        robotState3(1) = robotState3(1) + Ts*control(1)*cos(robotState3(3));
        robotState3(2) = robotState3(2) + Ts*control(1)*sin(robotState3(3));
    else
        robotState3(1) = robotState3(1) + control(1)/control(2)*(sin(control(2)*Ts + robotState3(3)) - sin(robotState3(3)));
        robotState3(2) = robotState3(2) - control(1)/control(2)*(cos(control(2)*Ts + robotState3(3)) - cos(robotState3(3)));
    end
    robotState3(3) = robotState3(3) + control(2)*Ts;
    
    stateSaved3 = [stateSaved3 robotState3];
end
%% Fifth spiral
alphaB =  alpha_conv -2*pi;
robotState4 = [stateSaved3(1,length(stateSaved3)); stateSaved3(2,length(stateSaved3)); stateSaved3(3,length(stateSaved3))];
stateSaved4 = robotState4;
beta = atan2(robotState4(2),robotState4(1));

while robotState4(1) > - 2*L
    Hwr = [ cos(robotState4(3)) -sin(robotState4(3)) robotState4(1);
        sin(robotState4(3)) cos(robotState4(3)) robotState4(2);
        0 0 1];
    
    spiralCenter_R = inv(Hwr)*[spiralCenter4;1];
    
    alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1)); 
    
    distance = sqrt(spiralCenter_R(1).^2 +spiralCenter_R(2).^2).^(1/2);
    distanceRef = distance;
    
    distance_d = - control(1)*cos(alpha);
    distanceRef_d = - control(1)*cos(alphaB);
    
    distance_k1 = distance + distance_d*Ts;
    
    epsilon = (-distance + distanceRef);
    
    if(abs(epsilon) > 1)
        epsilon = sign(epsilon);
    end;
    
    epsilon_d = (distance_d - distanceRef_d);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller Original
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    error = alpha - alphaB - alphaMax*epsilon;
    
    if (abs(alpha - alphaB) > 0.5*pi/180) && (reo)
          control(2) = lambda*(alpha - alphaB) + control(1)/distance*sin(alpha);
    else
        if(reo == 1)
            reo = 0;
            lambda = 0.02;
            %initialGap = abs(sqrt((robotState1(1)-spiralCenter1(1))^2 + (robotState1(2)-spiralCenter1(2))^2) - distanceRef);
        end
          control(2) = lambda*error + control(1)/distance*sin(alpha ) + alphaMax*epsilon_d;
    end;
           
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update robotState
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(control(2)) < 0.001)
        robotState4(1) = robotState4(1) + Ts*control(1)*cos(robotState4(3));
        robotState4(2) = robotState4(2) + Ts*control(1)*sin(robotState4(3));
    else
        robotState4(1) = robotState4(1) + control(1)/control(2)*(sin(control(2)*Ts + robotState4(3)) - sin(robotState4(3)));
        robotState4(2) = robotState4(2) - control(1)/control(2)*(cos(control(2)*Ts + robotState4(3)) - cos(robotState4(3)));
    end
    robotState4(3) = robotState4(3) + control(2)*Ts;
    
    stateSaved4 = [stateSaved4 robotState4];
end
%%
figure
hold on
grid on
plot(spiralCenter1(1), spiralCenter1(2), 'b+', 'LineWidth', 2)
plot(spiralCenter2(1),spiralCenter2(2), 'b+', 'LineWidth', 2)
plot(spiralCenter3(1),spiralCenter3(2), 'r+', 'LineWidth', 2)
plot(spiralCenter4(1),spiralCenter4(2), 'r+', 'LineWidth', 2)


plot(Center(1) , Center(2), 'redo', 'LineWidth', 2)
%plot(Center_bis(1), Center_bis(2), 'blacko', 'LineWidth', 2)
plot(rowlength -2*L, rowwidth/2 + d/2, 'black*')
plot( rowlength -2*L, -rowwidth/2 + d/2, 'black*')
plot( rowlength -2*L, -3*rowwidth/2 + d/2, 'black*')
lengthVec = 0.5;
robotRad = 0.25;
vecRob = 0:pi/720:2*pi;

for idx = 1 : 10 : size(stateSaved, 2)
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2);
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2);
    plot(stateSaved(1,idx)+robotRad*cos(vecRob), stateSaved(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2);
end

for idx = 1 : 10 : size(stateSaved1, 2)
    plot([stateSaved1(1,idx), stateSaved1(1,idx)+lengthVec*cos(stateSaved1(3, idx))], [stateSaved1(2,idx), stateSaved1(2,idx)+lengthVec*sin(stateSaved1(3, idx))], 'g', 'LineWidth',2);
    plot([stateSaved1(1,idx), stateSaved1(1,idx)+lengthVec*cos(stateSaved1(3, idx)+pi/2)], [stateSaved1(2,idx), stateSaved1(2,idx)+lengthVec*sin(stateSaved1(3, idx)+pi/2)], 'r', 'LineWidth',2);
    plot(stateSaved1(1,idx)+robotRad*cos(vecRob), stateSaved1(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2);
end

for idx = 1 : 10 : size(stateSaved2, 2)
    plot([stateSaved2(1,idx), stateSaved2(1,idx)+lengthVec*cos(stateSaved2(3, idx))], [stateSaved2(2,idx), stateSaved2(2,idx)+lengthVec*sin(stateSaved2(3, idx))], 'g', 'LineWidth',2);
    plot([stateSaved2(1,idx), stateSaved2(1,idx)+lengthVec*cos(stateSaved2(3, idx)+pi/2)], [stateSaved2(2,idx), stateSaved2(2,idx)+lengthVec*sin(stateSaved2(3, idx)+pi/2)], 'r', 'LineWidth',2);
    plot(stateSaved2(1,idx)+robotRad*cos(vecRob), stateSaved2(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2);
end

for idx = 1 : 10 : size(stateSaved3, 2)
    plot([stateSaved3(1,idx), stateSaved3(1,idx)+lengthVec*cos(stateSaved3(3, idx))], [stateSaved3(2,idx), stateSaved3(2,idx)+lengthVec*sin(stateSaved3(3, idx))], 'g', 'LineWidth',2);
    plot([stateSaved3(1,idx), stateSaved3(1,idx)+lengthVec*cos(stateSaved3(3, idx)+pi/2)], [stateSaved3(2,idx), stateSaved3(2,idx)+lengthVec*sin(stateSaved3(3, idx)+pi/2)], 'r', 'LineWidth',2);
    plot(stateSaved3(1,idx)+robotRad*cos(vecRob), stateSaved3(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2);
end

for idx = 1 : 10 : size(stateSaved4, 2)
    plot([stateSaved4(1,idx), stateSaved4(1,idx)+lengthVec*cos(stateSaved4(3, idx))], [stateSaved4(2,idx), stateSaved4(2,idx)+lengthVec*sin(stateSaved4(3, idx))], 'g', 'LineWidth',2);
    plot([stateSaved4(1,idx), stateSaved4(1,idx)+lengthVec*cos(stateSaved4(3, idx)+pi/2)], [stateSaved4(2,idx), stateSaved4(2,idx)+lengthVec*sin(stateSaved4(3, idx)+pi/2)], 'r', 'LineWidth',2);
    plot(stateSaved4(1,idx)+robotRad*cos(vecRob), stateSaved4(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2);
end
axis equal
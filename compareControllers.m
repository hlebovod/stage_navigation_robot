close all
clearvars
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sampling time
Ts = 0.1;
finalTime = 1000;
timeVec = 0 : Ts : finalTime;

% Initial robot state
state = [7; 0; pi];

% Control inputs
control = [0.2; 0];

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

choice = 3;

switch choice

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fred Controller 2 on differential robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 4
    
    % Servoing parameters
    lambda1 = 0.2;
    lambda2 = 5.0;
    alphaB = 16*pi/32;
    distanceRef = 4;
    
    
    % Spiral parameters
    spiralCenter = [0; 0];

    betaS = atan2(state(2)-spiralCenter(2), state(1)-spiralCenter(1));

    xs = distanceRef*cos(betaS);
    ys = distanceRef*sin(betaS);
    
    distance = ((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2)^(1/2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save display
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    nbSamples = length(timeVec);
    
    stateSaved = zeros(3, nbSamples);
    controlSaved = zeros(2, nbSamples);
    distanceSaved = zeros(1, nbSamples);
    distanceRefSaved = zeros(1, nbSamples);
    spiralSaved = zeros(2, nbSamples);
    
    alphaSaved = zeros(1, nbSamples - 1);
    errorSaved = zeros(2, nbSamples - 1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save Init
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    stateSaved(:,1) = state;
    controlSaved(:,1) = control;
    distanceSaved(:,1) = distance;
    distanceRefSaved(1,1) = distanceRef;
    spiralSaved(:,1) = [xs, ys]';
    
    for idx = 1 : nbSamples - 1
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Homogeneous transformation matrix
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        Hwr = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
        spiralCenter_R = Hwr \ [spiralCenter;1];

        alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1));

        distance = ((state(1) - spiralCenter(1)).^2 + (state(2) - spiralCenter(2)).^2).^(1/2);
        
        d_dot = -control(1)*cos(alpha);
        distance_k1 = distance + d_dot*Ts;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % controller
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % error on alpha
        e_d = distance - distanceRef; % error on the distance
        e_alpha = alpha - alphaB;
        
        e_d_dot = control(1)*(cos(alphaB) - cos(e_alpha + alphaB));
        
        control(2) = (control(1)*sin(e_alpha + alphaB))/(e_d + distanceRef) + (lambda1*e_d + lambda2*e_d_dot)/(control(1)*sin(e_alpha + alphaB));
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update state
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if(abs(control(2)) < 0.0001)
            state(1) = state(1) + Ts*control(1)*cos(state(3));
            state(2) = state(2) + Ts*control(1)*sin(state(3));
        else
            state(1) = state(1) + control(1)/control(2)*(sin(control(2)*Ts + state(3)) - sin(state(3)));
            state(2) = state(2) - control(1)/control(2)*(cos(control(2)*Ts + state(3)) - cos(state(3)));
        end
        state(3) = state(3) + control(2)*Ts;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update spiral analytic approach
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        beta = atan2(state(2)-spiralCenter(2), state(1)-spiralCenter(1));

        % BetaS is the angle between d(t1) et d(t2); it's not supposed to perform a
        % circle.
        betaS = acos((-(control(1)*Ts)^2 + distance^2 + distance_k1^2)/(2*distance*distance_k1)); % cosine law
        phi = pi-alphaB-betaS;
        d_Vs = distanceRef*sin(betaS)/sin(phi);
        distanceRef = distanceRef*sin(alphaB)/sin(phi); % sinus law

        Vs = d_Vs/Ts;

        xs = distanceRef*cos(beta);
        ys = distanceRef*sin(beta);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Save params
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        stateSaved(:,idx+1) = state;
        controlSaved(:,idx+1) = control;
        distanceSaved(1,idx+1) = distance;
        distanceRefSaved(1,idx+1) = distanceRef;
        spiralSaved(:,idx+1) = [xs, ys]';
        alphaSaved(1,idx) = alpha;
        errorSaved(:,idx) = [e_alpha, e_d];
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Display Results
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Display robot situation
    
    figure
    hold on
    axis equal
    title('robot pose')
    plot(spiralCenter(1), spiralCenter(2), 'k+', 'MarkerSize', 10, 'LineWidth', 2)
%     plot(stateSaved(1,1), stateSaved(2,1),'g+', 'MarkerSize', 10, 'LineWidth', 2)
%     plot(stateSaved(1,end), stateSaved(2,end),'r+', 'MarkerSize', 10, 'LineWidth', 2)
    
    plot(stateSaved(1,1:end), stateSaved(2,1:end), 'b','LineWidth', 2)
    plot(spiralSaved(1,1:end), spiralSaved(2,1:end), 'r:','LineWidth', 2)
    
    lengthVec = 0.5;    
    robotRad = 0.3;
    vecRob = 0:pi/720:2*pi;
    
    for idx = 1 : 200 : nbSamples
        plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
             [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
        plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
             [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)
        plot(stateSaved(1,idx)+robotRad*cos(vecRob), stateSaved(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2)

    %     plot([spiralCenter(1), spiralCenter(1) + distanceSaved(idx)*cos(betaSaved(idx))], [spiralCenter(2), spiralCenter(2) + distanceSaved(idx)*sin(betaSaved(idx))],'k', 'LineWidth', 1)
    end

    idx = nbSamples-1;
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
         [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
         [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)
     
%     % Display velocity
%     figure
%     hold on
%     title('velocity')
%     plot([timeVec(1,1), timeVec(1,end)], [vB, vB])
%     plot(timeVec(1,1:end), controlSaved(1,:))

    % Display distance
    figure
    hold on
    title('distance')
    plot(timeVec(1,1:end), distanceSaved, 'b','LineWidth', 2)
    plot(timeVec(1,1:end), distanceRefSaved, 'r:','LineWidth', 2)

    % Display alpha
    figure
    hold on
    title('alpha')
    plot([timeVec(1,1), timeVec(1,end)], [alphaB, alphaB]*180/pi)
    plot(timeVec(1,1:end-1), alphaSaved(1,:)*180/pi)

    % Display Lyapunov
    figure
    hold on
    title('Lyapunov')
    plot(timeVec(1,1:end-1), (errorSaved(1,1:end).^2 + errorSaved(2,1:end).^2)/2)
    
    
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fred Controller 1 on differential robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 3
    
    % Servoing parameters
    lambda_alpha = 0.2;
    lambda_d = 0.2;
    vB = 0.2;
    alphaB = 16*pi/32;
    distanceRef = 4;
    
    % Spiral parameters
    spiralCenter = [0; 0];

    betaS = atan2(state(2)-spiralCenter(2), state(1)-spiralCenter(1));

    xs = distanceRef*cos(betaS);
    ys = distanceRef*sin(betaS);
    
    distance = ((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2)^(1/2);
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save display
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    nbSamples = length(timeVec);
    
    stateSaved = zeros(3, nbSamples);
    controlSaved = zeros(2, nbSamples);
    distanceSaved = zeros(1, nbSamples);
    distanceRefSaved = zeros(1, nbSamples);
    spiralSaved = zeros(2, nbSamples);
    
    alphaSaved = zeros(1, nbSamples - 1);
    errorSaved = zeros(2, nbSamples - 1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save Init
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    stateSaved(:,1) = state;
    controlSaved(:,1) = control;
    distanceSaved(:,1) = distance;
    distanceRefSaved(1,1) = distanceRef;
    spiralSaved(:,1) = [xs, ys]';
    
%     nbSamples = 2;
    for idx = 1 : nbSamples - 1
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Homogeneous transformation matrix
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        Hwr = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
        spiralCenter_R = Hwr \ [spiralCenter;1];

        alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1));

        distance = ((state(1) - spiralCenter(1)).^2 + (state(2) - spiralCenter(2)).^2).^(1/2);
        
        d_dot = -control(1)*cos(alpha);
        distance_k1 = distance + d_dot*Ts;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % controller
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        e_alpha = alpha - alphaB; % error on alpha
        e_d = distance - distanceRef; % error on the distance
        
        control(1) = lambda_d*e_d + cos(alphaB)*vB/(cos(e_alpha + alphaB));
        control(2) = lambda_alpha*e_alpha + control(1)*sin(e_alpha + alphaB)/(e_d + distanceRef);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update state
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if(abs(control(2)) < 0.0001)
            state(1) = state(1) + Ts*control(1)*cos(state(3));
            state(2) = state(2) + Ts*control(1)*sin(state(3));
        else
            state(1) = state(1) + control(1)/control(2)*(sin(control(2)*Ts + state(3)) - sin(state(3)));
            state(2) = state(2) - control(1)/control(2)*(cos(control(2)*Ts + state(3)) - cos(state(3)));
        end
        state(3) = state(3) + control(2)*Ts;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update spiral analytic approach
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        beta = atan2(state(2)-spiralCenter(2), state(1)-spiralCenter(1));

        % BetaS is the angle between d(t1) et d(t2); it's not supposed to perform a
        % circle.
        betaS = real(acos((-(control(1)*Ts)^2 + distance^2 + distance_k1^2)/(2*distance*distance_k1))); % cosine law
        phi = pi-alphaB-betaS;
        d_Vs = distanceRef*sin(betaS)/sin(phi);
        distanceRef = distanceRef*sin(alphaB)/sin(phi); % sinus law

        Vs = d_Vs/Ts;

        xs = distanceRef*cos(beta);
        ys = distanceRef*sin(beta);
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Save params
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        stateSaved(:,idx+1) = state;
        controlSaved(:,idx+1) = control;
        distanceSaved(1,idx+1) = distance;
        distanceRefSaved(1,idx+1) = distanceRef;
        spiralSaved(:,idx+1) = [xs, ys]';
        alphaSaved(1,idx) = alpha;
        errorSaved(:,idx) = [e_alpha, e_d];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Display Results
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Display robot situation
    
    figure
    hold on
    title('robot pose')
    plot(spiralCenter(1), spiralCenter(2), 'k+', 'MarkerSize', 10, 'LineWidth', 2)
%     plot(stateSaved(1,1), stateSaved(2,1),'g+', 'MarkerSize', 10, 'LineWidth', 2)
%     plot(stateSaved(1,end), stateSaved(2,end),'r+', 'MarkerSize', 10, 'LineWidth', 2)
    
    plot(stateSaved(1,1:end), stateSaved(2,1:end), 'b','LineWidth', 2)
    plot(spiralSaved(1,1:end), spiralSaved(2,1:end), 'r:','LineWidth', 2)
    
    lengthVec = 0.5;    
    robotRad = 0.3;
    vecRob = 0:pi/720:2*pi;
    for idx = 1 : 200 : nbSamples
        plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
             [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
        plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
             [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)
        plot(stateSaved(1,idx)+robotRad*cos(vecRob), stateSaved(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2)

    %     plot([spiralCenter(1), spiralCenter(1) + distanceSaved(idx)*cos(betaSaved(idx))], [spiralCenter(2), spiralCenter(2) + distanceSaved(idx)*sin(betaSaved(idx))],'k', 'LineWidth', 1)
    end

    idx = nbSamples-1;
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
         [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
         [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)
     
    % Display velocity
    figure
    hold on
    title('velocity')
    plot([timeVec(1,1), timeVec(1,end)], [vB, vB])
    plot(timeVec(1,1:end), controlSaved(1,:))

    % Display distance
    figure
    hold on
    title('distance')
    plot(timeVec(1,1:end), distanceSaved, 'b','LineWidth', 2)
    plot(timeVec(1,1:end), distanceRefSaved, 'r:','LineWidth', 2)

    % Display alpha
    figure
    hold on
    title('alpha')
    plot([timeVec(1,1), timeVec(1,end)], [alphaB, alphaB]*180/pi)
    plot(timeVec(1,1:end-1), alphaSaved(1,:)*180/pi)

    % Display Lyapunov
    figure
    hold on
    title('Lyapunov')
    plot(timeVec(1,1:end-1), (errorSaved(1,1:end).^2 + errorSaved(2,1:end).^2)/2)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Original Controller on differential robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
case 2
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Servoing parameters
lambda = 0.2;
vB = 0.2;
alphaB = 17.0*pi/32;
distanceRef = 6;

% Spiral parameters
spiralCenter = [0; 0];

% Init parameters
distanceInit = sqrt((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2);

betaS = atan2(state(2)-spiralCenter(2), state(1)-spiralCenter(1));

xs = distanceRef*cos(betaS);
ys = distanceRef*sin(betaS);

if(distanceInit < distanceRef)
   alphaMax = (sign(alphaB)*pi - alphaB);
else
   alphaMax = alphaB;
end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nbSamples = length(timeVec);

stateSaved = zeros(3, nbSamples);
alphaSaved = zeros(1, nbSamples - 1);
alphaSSaved = zeros(1, nbSamples - 1);
omegaSaved = zeros(1, nbSamples - 1);
distanceSaved = zeros(1, nbSamples);
distanceRefsaved = zeros(1, nbSamples);
errorSaved = zeros(1, nbSamples - 1);
errorBSaved = zeros(1, nbSamples - 1);
errorSSaved = zeros(1, nbSamples - 1);
spiralSaved = zeros(2, nbSamples);
VrSaved = zeros(1, nbSamples);

x1Saved = zeros(1, nbSamples - 1);
x2Saved = zeros(1, nbSamples - 1);

betaSaved = zeros(1, nbSamples);
phiSaved = zeros(1, nbSamples);
thetaSSaved = zeros(1, nbSamples);
VsSaved = zeros(1, nbSamples);
distance_k1Saved = zeros(1, nbSamples);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stateSaved(:,1) = state;
distanceRefsaved(1,1) = distanceRef;
spiralSaved(:,1) = [xs, ys]';
distanceSaved(1,1) = distanceInit;
betaSaved(1,1) = betaS;
phiSaved(1,1) = pi - alphaB - betaS;
thetaSSaved(1,1) = pi + betaS - alphaB;
VrSaved(1,1) = control(1);
VsSaved(1,1) = control(1)';
distance_k1Saved(1,1) = distanceInit;


initialGap = abs(sqrt((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2) - distanceRef);
reo = 1; % reorientation: phase 1 then phase 2

for idx = 1 : nbSamples - 1

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Homogeneous transformation matrix
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        Hwr = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
        spiralCenter_R = Hwr \ [spiralCenter;1];

        alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1));

        distance = ((state(1) - spiralCenter(1)).^2 + (state(2) - spiralCenter(2)).^2).^(1/2);

        distance_d = - control(1)*cos(alpha);
        distanceRef_d = - control(1)*cos(alphaB);

        distance_k1 = distance + distance_d*Ts;

        epsilon = (-distance + distanceRef)/initialGap;

        if(abs(epsilon) > 1)
            epsilon = sign(epsilon);
        end;

        epsilon_d = (distance_d - distanceRef_d) / initialGap;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Controller Original
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        error = alpha - alphaB - alphaMax*epsilon;

        if (abs(alpha - alphaB) > 0.5*pi/180) && (reo)
    %         disp([num2str(idx) ' : Phase 1'])
            control(2) = lambda*(alpha - alphaB) + control(1)/distance*sin(alpha);
        else
            if(reo == 1)
                reo = 0;
                lambda = 0.02;
                swithiIdx = idx;
                initialGap = abs(sqrt((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2) - distanceRef);
            end
    %         disp([num2str(idx) ' : Phase 2'])
            control(2) = lambda*error + control(1)/distance*sin(alpha) + alphaMax*epsilon_d;
        end;

    %     disp(['Alpha error: ' num2str(abs(alpha - alphaB)*180/pi)])


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update state
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if(abs(control(2)) < 0.0001)
            state(1) = state(1) + Ts*control(1)*cos(state(3));
            state(2) = state(2) + Ts*control(1)*sin(state(3));
        else
            state(1) = state(1) + control(1)/control(2)*(sin(control(2)*Ts + state(3)) - sin(state(3)));
            state(2) = state(2) - control(1)/control(2)*(cos(control(2)*Ts + state(3)) - cos(state(3)));
        end
        state(3) = state(3) + control(2)*Ts;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update spiral analytic approach
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        beta = atan2(state(2)-spiralCenter(2), state(1)-spiralCenter(1));

        % BetaS is the angle between d(t1) et d(t2); it's not supposed to perform a
        % circle.
        betaS = acos((-(control(1)*Ts)^2 + distance^2 + distance_k1^2)/(2*distance*distance_k1)); % cosine law
        phi = pi-alphaB-betaS;
        d_Vs = distanceRef*sin(betaS)/sin(phi);
        distanceRef = distanceRef*sin(alphaB)/sin(phi); % sinus law

        Vs = d_Vs/Ts;

        xs = distanceRef*cos(beta);
        ys = distanceRef*sin(beta);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Save Data
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        stateSaved(:,idx+1) = state;
        alphaSSaved(1,idx) = alphaB + alphaMax*epsilon;
        alphaSaved(1,idx) = alpha;
        distanceSaved(1,idx+1) = distance;
        distanceRefsaved(1,idx+1) = distanceRef;
        errorSaved(1,idx) = error;
        errorBSaved(1,idx) = alpha - alphaB;
        errorSSaved(1,idx) = alphaMax*epsilon;
        omegaSaved(1,idx) = control(2);
        spiralSaved(:,idx+1) = [xs, ys]';

        x1Saved(1,idx) = sin(abs(alpha - alphaB)^2);
        x2Saved(1,idx) = (distance - distanceRef)^2 + sin(abs(alpha - alphaB)^2);

        betaSaved(1,idx) = betaS;
        phiSaved(1,idx) = phi;
        VrSaved(1,idx) = control(1);
        VsSaved(1,idx) = Vs;
        distance_k1Saved(1,idx) = distance_k1;
end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Alpha
alphaSSaved(1,1:swithiIdx) = NaN;

figure
hold on
title('Alpha')
set(gca,'fontsize',14)
xlabel('Time(s)', 'FontSize',14)
ylabel('\alpha(t) vs \alpha_B (deg)', 'FontSize',14)
plot([swithiIdx*Ts swithiIdx*Ts],[0 200], 'k')
plot(timeVec(:,1:end-1), alphaSaved*180/pi, 'b-', 'LineWidth',2)
plot([timeVec(:,1), timeVec(:,end-1)],[alphaB*180/pi alphaB*180/pi],'r:', 'LineWidth',2)
plot(timeVec(:,1:end-1), alphaSSaved*180/pi, 'k--', 'LineWidth',2)
legend('switch', 'alpha', 'alphaB', 'alphaS');

% Distance
figure
hold on
title('Distance')
set(gca,'fontsize',14)
xlabel('Time(s)', 'FontSize',14)
ylabel('d(t) vs d^*(t) (m)', 'FontSize',14)
plot([swithiIdx*Ts swithiIdx*Ts],[0 10], 'k')
plot(timeVec(:,1:end-1), distanceSaved(1,2:end), 'b-', 'LineWidth',2)
plot(timeVec(:,1:end-1), distanceRefsaved(1,2:end), 'r:', 'LineWidth',2)
legend('switch','distance', 'distanceRef');

% Error
VS = zeros(size(timeVec(:,1:end-1)));
VB = zeros(size(timeVec(:,1:end-1)));
VB(1: swithiIdx) = errorBSaved(1: swithiIdx).^2;
VB(swithiIdx+1:end) = NaN;
VS(1: swithiIdx) = NaN;
VS(swithiIdx+1:end) = errorSaved(swithiIdx+1:end).^2 + (distanceRefsaved(swithiIdx+2:end) - distanceSaved(swithiIdx+2:end)).^2;

maxValue = max([max(VS), max(VB)]);

figure
hold on
set(gca,'fontsize',14)
xlabel('Time(s)', 'FontSize',14)
ylabel('V_B(x_B(t))', 'FontSize',14)
title('Error')
plot(timeVec(:,1:end-1), VS, 'r:', 'LineWidth',2)
plot(timeVec(:,1:end-1), VB, 'b-', 'LineWidth',2)
plot([swithiIdx*Ts swithiIdx*Ts],[0 maxValue], 'k')

% Robot pose
figure
hold on
axis equal
set(gca,'fontsize',14)
xlabel('X(m)', 'FontSize',14)
ylabel('Y(m)', 'FontSize',14)
title('Robot position')
plot(spiralCenter(1), spiralCenter(2), 'k+', 'MarkerSize',10,'LineWidth',2);

plot(stateSaved(1,:), stateSaved(2,:), 'b-','LineWidth',2)
plot(spiralSaved(1,:), spiralSaved(2,:), 'r:', 'LineWidth',2)

% Display some robot situations along the navigation
lengthVec = 0.6;
robotRad = 0.3;
vecRob = 0:pi/720:2*pi;
for idx = 1 : 200 : nbSamples
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
         [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
    plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
         [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)
    plot(stateSaved(1,idx)+robotRad*cos(vecRob), stateSaved(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2)

    plot([spiralCenter(1), spiralCenter(1) + distanceSaved(idx)*cos(betaSaved(idx))], [spiralCenter(2), spiralCenter(2) + distanceSaved(idx)*sin(betaSaved(idx))],'k', 'LineWidth', 1)
end

idx = nbSamples-1;
plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
     [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
     [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)

 
% Angles Spiral
figure
hold on
title('Angles')
plot(timeVec(:,1:end-1), betaSaved(:,1:end-1)*180/pi)
plot(timeVec(:,1:end-1), phiSaved(:,1:end-1)*180/pi)
plot([timeVec(:,1), timeVec(:,end)], [alphaB, alphaB])
legend('betaS', 'phi', 'alphaB');

% Velocity robot/spiral
figure
hold on
title('Velocity')
plot(timeVec(1,1:end-1), VrSaved(1, 1:end-1), 'k:', 'LineWidth',2)
plot(timeVec(1,1:end-1), VsSaved(1, 1:end-1), 'b','LineWidth',2)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Original Controller 1 on differential robot (Spiral behavior)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

case 1
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Servoing parameters
lambda = 0.2;
alphaB = 17.0*pi/32;

% Spiral parameters
spiralCenter = [0; 0];

% Init parameters
distanceInit = sqrt((state(1)-spiralCenter(1))^2 + (state(2)-spiralCenter(2))^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nbSamples = length(timeVec);

stateSaved = zeros(3, nbSamples);
distanceSaved = zeros(1, nbSamples);
alphaSaved = zeros(1, nbSamples - 1);
errorSaved = zeros(1, nbSamples - 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stateSaved(:,1) = state;
distanceSaved(1,1) = distanceInit;

    for idx = 1 : nbSamples - 1

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Homogeneous transformation matrix
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        Hwr = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
        spiralCenter_R = Hwr \ [spiralCenter;1];

        alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1));

        distance = ((state(1) - spiralCenter(1)).^2 + (state(2) - spiralCenter(2)).^2).^(1/2);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Controller Original
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        error = alpha - alphaB;

        control(2) = lambda*error + control(1)/distance*sin(alpha);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update state
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if(abs(control(2)) < 0.0001)
            state(1) = state(1) + Ts*control(1)*cos(state(3));
            state(2) = state(2) + Ts*control(1)*sin(state(3));
        else
            state(1) = state(1) + control(1)/control(2)*(sin(control(2)*Ts + state(3)) - sin(state(3)));
            state(2) = state(2) - control(1)/control(2)*(cos(control(2)*Ts + state(3)) - cos(state(3)));
        end
        state(3) = state(3) + control(2)*Ts;


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Save Data
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        stateSaved(:,idx+1) = state;
        alphaSaved(1,idx) = alpha;
        distanceSaved(1,idx+1) = distance;
        errorSaved(1,idx) = error;
    end
    
    % Robot pose
    figure
    hold on
    axis equal
    set(gca,'fontsize',14)
    xlabel('X(m)', 'FontSize',14)
    ylabel('Y(m)', 'FontSize',14)
    title('Robot position')
    plot(spiralCenter(1), spiralCenter(2), 'k+', 'MarkerSize',10,'LineWidth',2);

    plot(stateSaved(1,:), stateSaved(2,:), 'b-','LineWidth',2)

    % Display some robot situations along the navigation
    lengthVec = 0.6;
    robotRad = 0.3;
    vecRob = 0:pi/720:2*pi;
    
    for idx = 1 : 200 : nbSamples
        plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx))], ...
             [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx))], 'g', 'LineWidth',2)
        plot([stateSaved(1,idx), stateSaved(1,idx)+lengthVec*cos(stateSaved(3, idx)+pi/2)], ...
             [stateSaved(2,idx), stateSaved(2,idx)+lengthVec*sin(stateSaved(3, idx)+pi/2)], 'r', 'LineWidth',2)
        plot(stateSaved(1,idx)+robotRad*cos(vecRob), stateSaved(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2)

    end
    
    % Alpha
    figure
    hold on
    title('Alpha')
    plot(timeVec(1,1:end-1), alphaSaved)
    plot([timeVec(1,1), timeVec(1,end-1)], [alphaB, alphaB])
    
    % Distance
    figure
    hold on
    title('Distance')
    plot(timeVec(1,:), distanceSaved)
    
    % Error
    figure
    hold on
    title('Error')
    plot(timeVec(1,1:end-1), errorSaved)
    
    % Lyapunov
    
    VB = errorSaved.^2/2;
    figure
    hold on
    title('Lyapunov')
    plot(timeVec(1,1:end-1), VB)

    
    
end
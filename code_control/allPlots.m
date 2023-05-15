function out = allPlots(robot,spiral,saved,timeVec)
   
%     stateSaved = savedParam{1};
%     controlSaved=  savedParam{2};
%     distanceSaved=  savedParam{3};
%     distanceRefSaved=  savedParam{4};
%     spiralSaved =  savedParam{5} ;
%     alphaSaved =  savedParam{6};
%     errorSaved = savedParam{7};
%     vB = savedParam{8};
%     alphaB = savedParam{9};

    nbSamples = length(timeVec);
    %%%%%%%%%%%%%%%%% Plot robot vectors
    lengthVec = 0.5;    
    robotRad = 0.3;
    vecRob = 0:pi/720:2*pi;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Display Results
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    % Display robot situation
     
    figure
    hold on
    axis equal
    title('robot pose')
    xlabel('X(m)', 'FontSize',14)
    ylabel('Y(m)', 'FontSize',14)
    plot(spiral.center(1), spiral.center(2), 'k+', 'MarkerSize', 10, 'LineWidth', 2)
     
    plot(saved.state(1,1:end), saved.state(2,1:end), 'b','LineWidth', 2)
    plot(saved.spiral(1,1:end), saved.spiral(2,1:end), 'r:','LineWidth', 2)
     
    lengthVec = 0.5;    
    robotRad = 0.3;
    vecRob = 0:pi/720:2*pi;
     
    for idx = 1 : 500 : nbSamples
        plot([saved.state(1,idx), saved.state(1,idx)+lengthVec*cos(saved.state(3, idx))], ...
             [saved.state(2,idx), saved.state(2,idx)+lengthVec*sin(saved.state(3, idx))], 'g', 'LineWidth',2)
        plot([saved.state(1,idx), saved.state(1,idx)+lengthVec*cos(saved.state(3, idx)+pi/2)], ...
             [saved.state(2,idx), saved.state(2,idx)+lengthVec*sin(saved.state(3, idx)+pi/2)], 'r', 'LineWidth',2)
        plot(saved.state(1,idx)+robotRad*cos(vecRob), saved.state(2,idx)+robotRad*sin(vecRob), 'k', 'LineWidth',2)
 
    %     plot([spiralCenter(1), spiralCenter(1) + distanceSaved(idx)*cos(betaSaved(idx))], [spiralCenter(2), spiralCenter(2) + distanceSaved(idx)*sin(betaSaved(idx))],'k', 'LineWidth', 1)
    end
 
    idx = nbSamples-1;
    plot([saved.state(1,idx), saved.state(1,idx)+lengthVec*cos(saved.state(3, idx))], ...
         [saved.state(2,idx), saved.state(2,idx)+lengthVec*sin(saved.state(3, idx))], 'g', 'LineWidth',2)
    plot([saved.state(1,idx), saved.state(1,idx)+lengthVec*cos(saved.state(3, idx)+pi/2)], ...
         [saved.state(2,idx), saved.state(2,idx)+lengthVec*sin(saved.state(3, idx)+pi/2)], 'r', 'LineWidth',2)
      
%     % Display velocity
%     figure
%     hold on
%     title('velocity')
%     xlabel('time(s)', 'FontSize',14)
%     ylabel('velocity(m.s^{-1})', 'FontSize',14)
%     plot([timeVec(1,1), timeVec(1,end)], [spiral.vB, spiral.vB])
%     plot(timeVec(1,1:end), saved.control(1,:))
%     legend('spiral speed','robot speed');
%  
%     % Display distance
%     figure
%     hold on
%     title('distance')
%     xlabel('time(s)', 'FontSize',14)
%     ylabel('distance(m)', 'FontSize',14)
%     plot(timeVec(1,1:end), saved.distance, 'b','LineWidth', 2)
%     plot(timeVec(1,1:end), saved.distanceRef, 'r:','LineWidth', 2)
%  
%     % Display alpha
%     figure
%     hold on
%     title('alpha')
%     plot([timeVec(1,1), timeVec(1,end)], [spiral.alpha, spiral.alpha]*180/pi)
%     plot(timeVec(1,1:end-1), saved.alpha(1,:)*180/pi)
%     
%     % Display errors
%  
    figure
    hold on
    subplot(1,2,1)
    plot(timeVec(1,1:end-1), rad2deg(saved.error(1,:)) , 'b','LineWidth', 2)
    title('alpha errors')
    xlabel('time(s)', 'FontSize',14)
    ylabel('degree(deg)', 'FontSize',14)
    
    
    subplot(1,2,2)
    plot(timeVec(1,1:end-1), saved.error(2,:), 'r','LineWidth', 2)
    title('distance errors')
    xlabel('time(s)', 'FontSize',14)
    ylabel('distance(m)', 'FontSize',14)
%     
%     % Display Lyapunov
%     figure
%     hold on
%     title('Lyapunov')
%     xlabel('time(s)', 'FontSize',14)
%     ylabel('1/2(e_{alpha}^2 + e_d^2)', 'FontSize',14)
%     plot(timeVec(1,1:end-1), (saved.error(1,1:end).^2 + saved.error(2,1:end).^2)/2)
%   
end


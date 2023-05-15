function [ robot,spiral,errors] = control(robot,spiral,lambda,Ts,choice)  
    % Robot alpha angle calculation
        Hwr = [cos(robot.theta) -sin(robot.theta) robot.x; sin(robot.theta) cos(robot.theta) robot.y; 0 0 1];
        spiralCenter_R = Hwr \ [spiral.center';1];
        robot.alpha =  atan2(spiralCenter_R(2), spiralCenter_R(1)) ;
        
   % Error computation
        e_alpha = robot.alpha - spiral.alpha;
        e_d = robot.distance - spiral.distance;
        errors = [e_alpha,e_d];
        
        e_d_dot = robot.v*(cos(spiral.alpha) - cos(e_alpha + spiral.alpha));      

        

 %%%%%%%%%%%%%%%%%%%%    Control laws      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch choice
    case 7   % Controller 7 : (sin(x) <= x * sing(x)
        robot.w = robot.v * sin(robot.alpha)/(robot.distance) +...
            e_d * robot.v *sign(robot.alpha) *(robot.alpha/2)*sign(e_alpha)+lambda.alpha(7)*e_alpha;
    case 6   % Controler 6 : e_alpha small
        robot.w = robot.v * sin(robot.alpha)/(robot.distance) + e_d * robot.v * sin(robot.alpha/2)+lambda.alpha(6)*e_alpha;
    case 5   % Controler 5 Frédéric 
        robot.v = cos(spiral.alpha) * spiral.vB /  cos(robot.alpha) + lambda.d(5) * sign(e_d) / cos (robot.alpha) ;
        robot.w = robot.v * sin(robot.alpha) / robot.distance + lambda.alpha(5) * sign( e_alpha);
    case 4   % Controler 4 Frédéric
        robot.w = robot.v*sin(robot.alpha) / robot.distance + ...
                (lambda.one*e_d + lambda.two*e_d_dot)/(robot.v*sin(robot.alpha) );
    case 3   % Controler 3 Frédéric
        robot.v = lambda.d(3)*e_d/cos(robot.alpha) + cos(spiral.alpha)*spiral.vB/cos(robot.alpha);
        robot.w = lambda.alpha(3)*e_alpha + robot.v*sin(robot.alpha)/(e_d + spiral.distance);
        
end
 %%%%%%%%%%%%%%%%%%%%    spiral update    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % Limitation on the speed
%      if(abs(robot.v) > 0.7 )
%          robot.v = 0.7*sign(robot.v) ;
%      end

    % State update
         [robot.x,robot.y,robot.theta] = stateUpdate( [robot.x,robot.y,robot.theta],[robot.v,robot.w],Ts);        
    
    % Spiral
        
        % Robot distance from the spiral at the next time instance :
        distance_k1 = norm([robot.x - spiral.center(1), robot.y - spiral.center(2)]);
        % Angle variation :
        betaS = real(acos((-(robot.v*Ts)^2 + robot.distance^2 + distance_k1^2)/(2*robot.distance*distance_k1)));
        if strcmp('right',spiral.direction)
            betaS = -betaS; % To change the direction of the spiral progression
        end
        % Spiral distance from the equiangular spiral formula :
        spiral.distance = spiral.distance*exp(cot(spiral.alpha)* ( -betaS ) );

        % Retrieving the polar robot coordinates :
        [beta, robot.distance] = cart2pol(robot.x ,robot.y);
        % From spiral distance and robot beta angle, find the carthesian
        % coordinate of the spiral :
        [spiral.x, spiral.y] = pol2cart(beta,spiral.distance);
        
      
        
end


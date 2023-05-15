close all
clearvars
clc
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.02;
finalTime = 200;
timeVec = 0 : Ts : finalTime;
nbSamples = length(timeVec);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot & Spiral Initi
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot = struct('x',6,...
               'y',1,...
               'theta',pi/2,...
               'alpha',[],...
               'v',0.1,...
               'w',[],...
               'distance',[],...
               'beta',[]...
               );

spiral = struct('x',[],...
                'y',[],...
                'center',[0 0],...
                'alpha', -15*pi/32,...
                'distance',4,...
                'vB',0.2,...
                'beta',[],...
                'direction','right'...  % Set the direction of the spiral. Left by default. 
                );
%                        1   2     3     4     5      6       7  
lambda = struct('alpha',[0   0    0.2    0    2      0.12     0],...
                'd',    [0   0    0.8    0    0.2    0        0], ...
                'one',0.005,...
                'two',0.09...
                );
            
    % Initiate the robot and spiral

[robot.beta, robot.distance] = cart2pol(robot.x,robot.y);
spiral.x = spiral.distance*cos(robot.beta);
spiral.y = spiral.distance*sin(robot.beta);

    

    % Save Init
saved = savedInit(robot,spiral,nbSamples);

choice  = 6;
for idx = 1 : nbSamples - 1
    
 
    [robot,spiral,errors] = control(robot,spiral,lambda,Ts,choice);
        
   
    saved.state(:,idx+1) = [robot.x,robot.y,robot.theta];
    saved.control(:,idx+1) = [robot.v,robot.w];
    saved.distance(1,idx+1) = robot.distance;
    saved.distanceRef(1,idx+1) = spiral.distance;
    saved.spiral(:,idx+1) = [spiral.x,spiral.y]';
    saved.alpha(1,idx) = robot.alpha;
    saved.error(:,idx) = errors;

end

allPlots(robot,spiral,saved,timeVec)



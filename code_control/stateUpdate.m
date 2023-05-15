function [x,y,theta] = stateUpdate( state, control, Ts )
% stateUpdate ... update the states ! 

    if(abs(control(2)) < 0.0001)
        x = state(1) + Ts*control(1)*cos(state(3));
        y = state(2) + Ts*control(1)*sin(state(3));
    else
        x = state(1) + control(1)/control(2)*(sin(control(2)*Ts + state(3)) - sin(state(3)));
        y = state(2) - control(1)/control(2)*(cos(control(2)*Ts + state(3)) - cos(state(3)));
    end
    theta = state(3) + control(2)*Ts;

end

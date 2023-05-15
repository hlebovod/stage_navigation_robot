function [x, y, phi] = compute_pose(r1, alpha1, r2, alpha2, L)

    beta = acos((r2^2 - r1^2 - L^2 )/(-2*L*r1));
    phi = alpha1 - beta;
    omega = alpha2 - alpha1;
    
    if omega < 0 
        x = -r1*cos(beta);
        y = r1*sin(beta);
    else
        x = r1*cos(beta);
        y = r1*sin(beta);
    end
    
end


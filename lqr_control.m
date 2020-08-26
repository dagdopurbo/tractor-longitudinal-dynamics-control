function xdot = lqr_control(tspan,x,A,B,F,K,r,w,dt)
% NOTES:
% r is the control reference (yaw rate)
% u is the control input (steering angle)
% K is the LQR gain
% w is the plant disturbance
    
% steering angle/control input
    if length(r) > 1
        u = F*r(floor(tspan*(1/dt))+1,:)-K*x;
    else
        u = F*r-K*x;
    end

% Yaw rate control
    xdot = A*x + B*u + B*w(floor(tspan*(1/dt))+1,:);
end

function x_hat = kalman_estimator(tspan,x,y,A,B,C,K,L,dt)
% NOTES:
% r is the control reference (yaw rate)
% u is the control input (steering angle)
% K is the LQR gain
% w is the plant disturbance
% L is Kalman gain
% y is measurement output

% Control input
    u = -K*x;
% Estimated states
    x_hat = A*x + B*u + L*(y(:,floor(tspan*(1/dt))+1) - C*x);
end
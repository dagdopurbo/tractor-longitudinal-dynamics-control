function xdot = lateral_dynamics(x,u)
% NOTES:
% System states:
% x(1) is lateral speed vy (m/s) and x(2) is yaw rate w (rad/s)
% System control input:
% u is steering angle (degree)

% Lateral dynamic state space    
    xdot = A*x + B*u;    
end
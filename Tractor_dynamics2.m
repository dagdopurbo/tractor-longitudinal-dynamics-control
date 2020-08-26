clc; clear; close all;

%% time parameter
dt = 0.1;
t0 = 0;
tf = 51-dt;
tspan = (t0:dt:tf)';
%% Define the tractor parameters
Lf = 1.25;
Lr = 1.25;
wheel_rad = 0.6;
m = 650; % Kg
Iz = 1100; % Kg.m
Caf = 120; % N/deg
Car = 100; % N/deg

%% Define way points and initial position
% waypoints = [0,0; 0,50; 50,50; 50,0];
n = (0:2.6:50)';
% Reference waypoints
xRef = [zeros(length(n)-1,1);1*n(1:end-1); n(end)*ones(length(n),1);1*flipud(n)];
yRef = [1*n(1:end-1); n(end)*ones(length(n)-1,1); 1*flipud(n); zeros(length(n),1)];
thetaRef = [deg2rad(90)*ones(length(n)-1,1);...
          deg2rad(0)*ones(length(n)-1,1);...
          deg2rad(-90)*ones(length(n)-1,1);...
          deg2rad(-180)*ones(length(n)+1,1)];
waypoints = [xRef yRef];

initialPosition = [5;-10;deg2rad(-90)];
xAct(1,:) = initialPosition(1);
yAct(1,:) = initialPosition(2);
thetaAct(1,:) = initialPosition(3);

vMax = 5;
wMax = 0.75;
vDes = 5;
wDes = 0;
%% Kinematics controller Gain
k1 = 4;
k2 = 2;
k3 = 5;

%% Parameter LQR
initialx = [0;0];
x(:,1) = initialx;
tlqr = (0:dt:0.5)';
% noise
n = length(tlqr);
Sw = 0.1; Sv = 0.01;
w = Sw*randn(n,1); % add disturbance
v = Sv*randn(n,1); % add noise

%% index toko indonesia
k = 1;
i = 1;
%% Simulation
viz = Visualizer2D;
 while tspan(k,:)<tf    
%% Kinematics controller

% Waypoints setup
    if i == 1
        xn = (xAct(k,:) - xRef(i,:))*cos(thetaRef(i,:)) + (yAct(k,:) - yRef(i,:))*sin(thetaRef(i,:));
    elseif i == length(xRef)
        xn = (xAct(k,:) - xRef(i,:))*cos(thetaRef(i,:)) + (yAct(k,:) - yRef(i,:))*sin(thetaRef(i,:));
    else
        xn = (xAct(k,:) - xRef(i,:))*cos((thetaRef(i,:)+thetaRef(i-1,:))/2) + (yAct(k,:) - yRef(i,:))*sin((thetaRef(i,:)+thetaRef(i-1,:))/2);
    end
    
% Error Calculation
    if xn < 0
        xe(k,:) = (xRef(i,:) - xAct(k,:))*cos(thetaAct(k,:)) + (yRef(i,:) - yAct(k,:))*sin(thetaAct(k,:));
        ye(k,:) = -(xRef(i,:) - xAct(k,:))*sin(thetaAct(k,:)) + (yRef(i,:) - yAct(k,:))*cos(thetaAct(k,:));
        thetae(k,:) = thetaRef(i,:) - thetaAct(k,:);
    else
        if i < length(xRef)
            i = i + 1;
        else
            i = length(xRef);
        end
        xe(k,:) = (xRef(i,:) - xAct(k,:))*cos(thetaAct(k,:)) + (yRef(i,:) - yAct(k,:))*sin(thetaAct(k,:));
        ye(k,:) = -(xRef(i,:) - xAct(k,:))*sin(thetaAct(k,:)) + (yRef(i,:) - yAct(k,:))*cos(thetaAct(k,:));
        thetae(k,:) = thetaRef(i,:) - thetaAct(k,:);
    end
    
% Lyapunov Controller
    vRef(k,:) = k1*xe(k,:) + vDes*cos(thetae(k,:));
    if vRef(k,:) > vMax
        vRef(k,:) = vMax;
    elseif vRef(k,:) < -vMax
        vRef(k,:) = -vMax;
    else
        % do nothing
    end
    
    wRef(k,:) = (1/k2)*ye(k,:)*vDes + k3*sin(thetae(k,:)) + wDes;
    if wRef(k,:) > wMax
        wRef(k,:) = wMax;
    elseif wRef(k,:) < -wMax
        wRef(k,:) = -wMax;
    else
        % do nothing
    end
    
    vx = abs(vRef(k,:));
%% Dynamics
% State space transition matrix
     A(1,1) = -(Caf + Car)/(m*vx);
     A(1,2) = (-Lf*Caf + Lr*Car)/(m*vx) - vx;
     A(2,1) = (-Lf*Caf + Lr*Car)/(Iz*vx);
     A(2,2) = -(Lf^2*Caf + Lr^2*Car)/(Iz*vx);
     % Control input matrix
     B(1,1) = Caf/m;
     B(2,1) = Caf*Lf/Iz;
     % Output transition matrix
     C = [0 1];
     D = 0;
%% Linear Quadratic Regulator
     ref = wRef(k,:);
     % LQR
     Q = C'*C; Q(2,2) = 100;
     R = 0.0001;
     K = lqr(A,B,Q,R);
     Ts = ss(A-B*K,B,C,D);
     F = 1/dcgain(Ts);
     [tm,xm] = ode45(@(t,x)lqr_control(t,x,A,B,F,K,ref,w,dt),tlqr,initialx);
     ym(k,:) = C*xm(end,:)' + v(end)';
     
     % actual omega
     wAct(k,:) = ym(k,:);
     % acrual speed
     vAct(k,:) = vRef(k,:);
     %% Kinematics
     % Robot Inverse Kinematics
     if abs(vAct(k,:)) > 1e-12
        delta_f(k,:) = atan(wAct(k,:)*(Lf+Lr)/vAct(k,:));
     else
        delta_f(k,:) = 0;
     end

     if delta_f(k,:) > deg2rad(30)
         delta_f(k,:) = deg2rad(30);
     elseif delta_f(k,:) < deg2rad(-30)
         delta_f(k,:) = deg2rad(-30);
     else
        % do nothing
     end
     delta_r = 0;

     w_f = vAct(k,:)/(wheel_rad*cos(delta_f(k,:)));
     w_r = vAct(k,:)/wheel_rad;
%      w_r = w_f;


     % Robot Forward Kinematics
     vxAct = (wheel_rad/2)*(w_f*cos(delta_f(k,:)) + w_r*cos(delta_r));
     vyAct = (wheel_rad/2)*(w_f*sin(delta_f(k,:)) + w_r*sin(delta_r));
     omegaAct = (wheel_rad/(Lf+Lr))*(w_f*sin(delta_f(k,:)) - w_r*sin(delta_r));

     % Conversion of local to global frame
     xdotAct(k,:) = vxAct*cos(thetaAct(k,:));% - vyAct*sin(thetaAct(k,:));
     ydotAct(k,:) = vxAct*sin(thetaAct(k,:));% + vyAct*cos(thetaAct(k,:));
     thetadotAct(k,:) = omegaAct;
 
     % Discrete integration
     xAct(k+1,:) = xAct(k,:) + xdotAct(k,:)*dt;
     yAct(k+1,:) = yAct(k,:) + ydotAct(k,:)*dt;
     thetaAct(k+1,:) = thetaAct(k,:) + thetadotAct(k,:)*dt;
     if thetaAct(k+1,:) < 2*pi
        % do nothing
     else
        thetaAct(k+1,:) = thetaAct(k+1,:) - 2*pi;
     end
     
     viz([xAct(k+1,:); yAct(k+1,:); thetaAct(k+1,:)]), 
     axis([-10 60 -20 60])
     
% Expected variables to VREP
     steeringAngle(k,:) = delta_f(k,:);
     k = k+1;
     pause(0.05)
end
 
figure,plot(waypoints(:,1),waypoints(:,2),'--r'), hold on,
plot(xAct,yAct,'-b'), grid on, legend('reference','actual')
xlabel('X (m)'), ylabel('Y (m)'),

figure,plot(tspan(1:end-1),wRef,'--r',tspan(1:end-1),wAct,'-b'),grid on
xlabel('time(sec)'),ylabel('omega(rad/s)'),legend('reference','actual')

figure,plot(tspan(1:end-1),steeringAngle),grid on
xlabel('time(sec)'),ylabel('steering angle(rad)'),
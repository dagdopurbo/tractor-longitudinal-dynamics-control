clc; clear; close all 
%% Control Parameters
dt = 0.1; % seconds --> sampling time
% tspan = 0:dt:30-dt; tspan = tspan';
tspan = (0:dt:0.5)';
%% Parameters of Fendt Tractor Model Vario 939
% rated speed 2100 RPM
% Max Torque 1565 Nm
Lf = 1.57; % m
Lr = 3.2; % m
m = 11000; % Kg
Iz = 18500; % Kg.m
Caf = 1400; % N/deg
Car = 3000; % N/deg
vx = 2.5; % m/s (This value should be the output of longitudinal dynamic)

%% Lateral dynamics
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
%% Continuous Time Open Loop Transfer Function (S-Domain)
[numS,denS] = ss2tf(A,B,C,D);
Gs = tf(numS,denS);
figure,rlocus(Gs),title('Open-loop root locus'),grid on
figure,step(Gs),title('Open-loop impulse response'),grid on
Gs_rank = rank(ctrb(A,B));

if Gs_rank == 2 
    disp('the system is controllable')
else
    disp('the system is uncontrollable')
end

%% Reference
% r = [5*ones(size(tspan(1:1000)))' 3*ones(size(tspan(1:1000)))' 6*ones(size(tspan(1:1000)))']';
r = -7.5*ones(size(tspan));

%% Linear Quadratic Regulator
% noise
n = length(tspan);
% Sw = 0.1; Sv = 0.01;
Sw = 0; Sv = 0;
w = Sw*randn(n,1); % add disturbance
v = Sv*randn(n,1); % add noise
% LQR
Q = C'*C; Q(2,2) = 100;
R = 0.0001;
K = lqr(A,B,Q,R);
Ts = ss(A-B*K,B,C,D);
F = 1/dcgain(Ts);
x0 = [0;0];
[tm,xm] = ode45(@(t,x)lqr_control(t,x,A,B,F,K,r',w,dt),tspan,x0);
ym = C*xm' + v';
figure, plot(tm,ym,tspan,r),grid on,legend('actual','reference')
xlabel('time(sec)'),ylabel('degree/sec')

%% Error
error = ym - r';
figure,plot(tspan,error),grid on,
xlabel('time(sec)'),ylabel('amplitude')
mean_error = mean(error);
% =========================================================================
% Integral Tracker Design for System with Added Zero at s=0
% =========================================================================


clear;
clc;
close all;

%% ------------------------------------------------------------------------
% Physical Parameters
%% ------------------------------------------------------------------------
M = 2.0;
m = 0.2;
l = 0.5;
g = 9.81;
I = 0.099;

%% ------------------------------------------------------------------------
% System Parameters
%% ------------------------------------------------------------------------
Delta = I*(M+m) + M*m*l^2;

a  = -(g*m^2*l^2)/Delta;
b  = ((M+m)*g*m*l)/Delta;

B2 = I/Delta;
B4 = -m*l/Delta;

%% ------------------------------------------------------------------------
% Reduced-order model
% xr = [vx theta omega]'
%% ------------------------------------------------------------------------
Ar = [0  a  0;
      0  0  1;
      0  b  0];

Br = [B2;
      0;
      B4];

Cr = [1 0 0];

%% ------------------------------------------------------------------------
% Augmented System
%% ------------------------------------------------------------------------
Aa = [Ar zeros(3,1);
     -Cr       0];

Ba = [Br;
      0];

%% Check controllability
fprintf('Rank of augmented controllability matrix = %d\n', ...
    rank(ctrb(Aa,Ba)));

if rank(ctrb(Aa,Ba)) ~= size(Aa,1)
    error('Augmented system is NOT controllable.');
end

%% ------------------------------------------------------------------------
% LQR Design
%% ------------------------------------------------------------------------
Q = diag([10 1000 10 5000]);
R = 1;

Ka = lqr(Aa,Ba,Q,R);

K  = Ka(1:3);
Ki = Ka(4);

disp('State-feedback gain K = ');
disp(K);

disp('Integral gain Ki = ');
disp(Ki);

%% ------------------------------------------------------------------------
% Closed-loop System
%% ------------------------------------------------------------------------

Acl = [Ar-Br*K   -Br*Ki;
      -Cr          0];

Bcl = [zeros(3,1);
       1];

Ccl = [1 0 0 0;           % velocity
       0 1 0 0;           % angle
      -K  -Ki];           % control force

Dcl = [0;
       0;
       0];

sysCL = ss(Acl,Bcl,Ccl,Dcl);

%% ------------------------------------------------------------------------
% Simulation
%% ------------------------------------------------------------------------

t = 0:0.01:5;

r = ones(size(t));      % 1 m/s velocity reference

[Y,t,X] = lsim(sysCL,r,t);

%% ------------------------------------------------------------------------
% Plots
%% ------------------------------------------------------------------------

figure;

subplot(3,1,1)
plot(t,r,'k--','LineWidth',1.5)
hold on
plot(t,Y(:,1),'b','LineWidth',2)
grid on
title('Velocity Tracking')
ylabel('Velocity (m/s)')
legend('Reference','Velocity','Location','best')

subplot(3,1,2)
plot(t,Y(:,2),'r','LineWidth',2)
grid on
title('Pendulum Angle')
ylabel('\theta (rad)')

subplot(3,1,3)
plot(t,Y(:,3),'LineWidth',2)
grid on
title('Control Force')
xlabel('Time (s)')
ylabel('Force (N)')
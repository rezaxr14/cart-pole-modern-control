% =========================================================================
% MATLAB Code: Full-Order Observer Design and Simulation
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
% System Matrices
%% ------------------------------------------------------------------------
Delta = I*(M+m) + M*m*l^2;

a  = -(g*m^2*l^2)/Delta;
b  = ((M+m)*g*m*l)/Delta;

B2 = I/Delta;
B4 = -m*l/Delta;

A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1;
     0 0 b 0];

B = [0;
     B2;
     0;
     B4];

C = [1 0 0 0;
     0 0 1 0];

Dsys = zeros(2,1);

%% ------------------------------------------------------------------------
% Check Observability
%% ------------------------------------------------------------------------
Ob = obsv(A,C);

fprintf('Rank of Observability Matrix = %d\n',rank(Ob));

if rank(Ob) ~= size(A,1)
    error('The system is NOT observable.');
end

disp('The system is fully observable.');

%% ------------------------------------------------------------------------
% Observer Design
%% ------------------------------------------------------------------------
observer_poles = [-10 -12 -14 -16];

L = place(A',C',observer_poles)';

disp('Observer Gain L = ');
disp(L);

disp('Observer poles:')
disp(eig(A-L*C))

%% ------------------------------------------------------------------------
% Augmented Plant + Observer
%% ------------------------------------------------------------------------
Aaug = [A              zeros(4);
        L*C            A-L*C];

Baug = [B;
        B];

Caug = eye(8);

Daug = zeros(8,1);

sysAug = ss(Aaug,Baug,Caug,Daug);

%% ------------------------------------------------------------------------
% Simulation
%% ------------------------------------------------------------------------

x0      = [0.10;
           0;
           0.10;
           0];

xhat0   = zeros(4,1);

x0aug = [x0;
         xhat0];

t = 0:0.001:1;

u = 0.5*ones(size(t));

[Y,t,X] = lsim(sysAug,u,t,x0aug);

x     = X(:,1:4);
xhat  = X(:,5:8);

e = x-xhat;

%% ------------------------------------------------------------------------
% Plot Actual vs Estimated States
%% ------------------------------------------------------------------------

figure;

subplot(2,1,1)

plot(t,x(:,2),'b','LineWidth',2)
hold on
plot(t,xhat(:,2),'r--','LineWidth',2)

grid on
title('Cart Velocity')
xlabel('Time (s)')
ylabel('m/s')

legend('Actual','Estimated','Location','best')

subplot(2,1,2)

plot(t,x(:,4),'b','LineWidth',2)
hold on
plot(t,xhat(:,4),'r--','LineWidth',2)

grid on
title('Pendulum Angular Velocity')
xlabel('Time (s)')
ylabel('rad/s')

legend('Actual','Estimated','Location','best')

%% ------------------------------------------------------------------------
% Estimation Error
%% ------------------------------------------------------------------------

figure

plot(t,e(:,1),'LineWidth',1.5)
hold on
plot(t,e(:,2),'LineWidth',1.5)
plot(t,e(:,3),'LineWidth',1.5)
plot(t,e(:,4),'LineWidth',1.5)

grid on

title('Observer Estimation Errors')

xlabel('Time (s)')
ylabel('Error')

legend('e_x','e_v','e_\theta','e_\omega','Location','best')
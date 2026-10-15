% =========================================================================
% MATLAB Code for State Transition Matrix and System Response
% =========================================================================

clear;
clc;
close all;

%% ------------------------------------------------------------------------
% Physical Parameters
%% ------------------------------------------------------------------------
M = 2.0;      % Cart mass (kg)
m = 0.2;      % Pendulum mass (kg)
l = 0.5;      % Distance to pendulum COM (m)
g = 9.81;     % Gravity (m/s^2)
I = 0.099;    % Pendulum inertia (kg.m^2)

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
% Part A : State Transition Matrix
%% ------------------------------------------------------------------------
disp('====================================================');
disp('State Transition Matrix');
disp('====================================================');

syms s t

Isym = sym(eye(4));

Phi_s = simplify(inv(s*Isym - A));

Phi_t = simplify(ilaplace(Phi_s,s,t));

disp('Phi(s) =');
disp(Phi_s);

disp('Phi(t) =');
disp(Phi_t);

%% ------------------------------------------------------------------------
% Part B : Time Response
%% ------------------------------------------------------------------------
disp('====================================================');
disp('System Response');
disp('====================================================');

sys = ss(A,B,C,Dsys);

% Initial condition
x0 = [0.10;
      0;
      0.05;
      0];

% Simulation time
t = 0:0.01:2;

% Unit-step input
u = ones(size(t));

% Simulation
[y,t,x] = lsim(sys,u,t,x0);

%% ------------------------------------------------------------------------
% Part C : Plot States
%% ------------------------------------------------------------------------
figure;

plot(t,x(:,1),'LineWidth',1.5);
hold on;
plot(t,x(:,2),'LineWidth',1.5);
plot(t,x(:,3),'LineWidth',1.5);
plot(t,x(:,4),'LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('States');
title('State Responses');

legend('x','\dot{x}','\theta','\dot{\theta}', ...
    'Location','best');

%% ------------------------------------------------------------------------
% Plot Outputs
%% ------------------------------------------------------------------------
figure;

plot(t,y(:,1),'LineWidth',2);
hold on;
plot(t,y(:,2),'LineWidth',2);

grid on;
xlabel('Time (s)');
ylabel('Outputs');
title('Output Responses');

legend('Cart Position','Pendulum Angle',...
    'Location','best');
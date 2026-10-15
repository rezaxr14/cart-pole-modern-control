% =========================================================================
% MATLAB Code: Closed-Loop Simulation
% Unity Output Feedback vs. Full-State LQR Feedback
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

sys_open = ss(A,B,C,Dsys);

%% ------------------------------------------------------------------------
% Initial Conditions
%% ------------------------------------------------------------------------
x0 = [0.10;
      0;
      0.10;
      0];

t = 0:0.01:5;

%% ========================================================================
% CASE A : Unity Output Feedback
% u = -theta
%% ========================================================================

Cy = C(2,:);          % Feedback only pendulum angle
Kunity = 1;           % Unity gain

Acl_unity = A - B*Kunity*Cy;

sys_unity = ss(Acl_unity,B,C,Dsys);

[Y_unity,t_unity,X_unity] = initial(sys_unity,x0,t);

%% ========================================================================
% CASE B : Full-State Feedback (LQR)
%% ========================================================================

Q = diag([10 1 100 1]);
R = 0.1;

Klqr = lqr(A,B,Q,R);

Acl_lqr = A - B*Klqr;

sys_lqr = ss(Acl_lqr,B,C,Dsys);

[Y_lqr,t_lqr,X_lqr] = initial(sys_lqr,x0,t);

%% ------------------------------------------------------------------------
% Closed-loop poles
%% ------------------------------------------------------------------------

disp('Open-loop poles:')
disp(eig(A))

disp('Unity-feedback poles:')
disp(eig(Acl_unity))

disp('LQR poles:')
disp(eig(Acl_lqr))

%% ------------------------------------------------------------------------
% Plot Results
%% ------------------------------------------------------------------------

figure;

subplot(2,1,1)

plot(t_unity,X_unity(:,1),'b','LineWidth',1.5)
hold on
plot(t_unity,X_unity(:,3),'r','LineWidth',1.5)

grid on

title('Unity Output Feedback')
xlabel('Time (s)')
ylabel('States')

legend('Cart Position','Pendulum Angle','Location','best')

subplot(2,1,2)

plot(t_lqr,X_lqr(:,1),'b','LineWidth',1.5)
hold on
plot(t_lqr,X_lqr(:,3),'r','LineWidth',1.5)

grid on

title('Full-State Feedback (LQR)')
xlabel('Time (s)')
ylabel('States')

legend('Cart Position','Pendulum Angle','Location','best')
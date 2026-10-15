% =========================================================================
% MATLAB Code for State Transition Matrix and System Response
% =========================================================================

clear; clc; close all;

% --- 1. Define Physical Parameters ---
M = 2.0;    % Mass of the cart (kg)
m = 0.2;    % Mass of the pendulum (kg)
l = 0.5;    % Distance to the center of mass of the pendulum (m)
g = 9.81;   % Gravity (m/s^2)
I = 0.099;  % Moment of inertia of the pendulum (kg.m^2)

% --- 2. Calculate Matrix Elements ---
D = I*(M + m) + M*m*l^2;
a = (g * m^2 * l^2) / D;
b = -((M + m) * g) / D;
B2 = (I + m*l^2) / D;
B4 = -(m * l) / D;

% --- 3. Build State-Space Matrices ---
A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1;
     0 0 b 0];

B = [0; B2; 0; B4];

C = [1 0 0 0;
     0 0 1 0];
     
D_sys = [0; 0];

%% --- Part A: State Transition Matrix (Symbolic) ---
disp('--- 1. State Transition Matrix (Phi) ---');
% Note: This section requires the Symbolic Math Toolbox
syms s t real

I_mat = eye(4);
% Compute (sI - A)^-1
Phi_s = inv(s*I_mat - A);

% Inverse Laplace to get Phi(t)
Phi_t = ilaplace(Phi_s, s, t);

disp('State Transition Matrix Phi(t) = ');
disp(Phi_t);

%% --- Part B: System Response (Time Domain Simulation) ---
disp('--- 2. System Response to Initial Conditions and Step Input ---');

% Define arbitrary initial conditions
% x0 = [Position; Velocity; Angle; Angular Velocity]
x0 = [0.1; 0; 0.05; 0]; % e.g., 0.1m cart displacement, 0.05 rad initial angle

% Define time vector for simulation
% Since the system is unstable, we use a short time duration (e.g., 2 seconds)
t_sim = 0:0.01:2; 

% Define Unit Step Input (u(t) = 1)
u = ones(size(t_sim));

% Create state-space model object
sys = ss(A, B, C, D_sys);
sys.StateName = {'x', 'x_dot', 'theta', 'theta_dot'};
sys.OutputName = {'Cart Position', 'Pendulum Angle'};

% Simulate total response (forced + natural) using lsim
% y_out contains outputs, x_state contains all 4 states
[y_out, t_out, x_state] = lsim(sys, u, t_sim, x0);

%% --- Part C: Plotting the Results ---
figure('Name', 'System Responses', 'Position', [100, 100, 800, 600]);

% 1. Plot State Responses
subplot(2,1,1);
plot(t_out, x_state(:,1), 'LineWidth', 1.5); hold on;
plot(t_out, x_state(:,2), 'LineWidth', 1.5);
plot(t_out, x_state(:,3), 'LineWidth', 1.5);
plot(t_out, x_state(:,4), 'LineWidth', 1.5);
title('State Responses (x(t)) to Step Input & Initial Conditions');
xlabel('Time (sec)');
ylabel('Amplitude');
legend('x (Position)', 'v (Velocity)', '\theta (Angle)', '\omega (Angular Vel)', 'Location', 'best');
grid on;

% 2. Plot Output Responses
subplot(2,1,2);
plot(t_out, y_out(:,1), 'b', 'LineWidth', 2); hold on;
plot(t_out, y_out(:,2), 'r', 'LineWidth', 2);
title('Output Responses (y(t))');
xlabel('Time (sec)');
ylabel('Amplitude');
legend('Output 1: Cart Position (x)', 'Output 2: Pendulum Angle (\theta)', 'Location', 'best');
grid on;
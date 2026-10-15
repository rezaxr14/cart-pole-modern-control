% =========================================================================
% MATLAB Code: Closed-Loop Simulation (Unity Feedback vs. State Feedback)
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

% --- 3. Build Open-Loop State-Space ---
A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1;
     0 0 b 0];

B = [0; B2; 0; B4];

C = [1 0 0 0;
     0 0 1 0];
     
D_sys = [0; 0];
sys_open = ss(A, B, C, D_sys);
sys_open.StateName = {'x', 'x_dot', 'theta', 'theta_dot'};

% --- 4. Define Initial Conditions ---
% x0 = [Position; Velocity; Angle; Angular Velocity]
x0 = [0.1; 0; 0.1; 0]; 
t = 0:0.01:3; % Simulation time (3 seconds)

%% --- CASE A: Unity Feedback (On Pendulum Angle) ---
% We feedback only the second output (Angle) to the input.
% Feedback matrix K_unity = [0, 1] meaning u = - (0*x + 1*theta)
K_unity = [0, 1]; 
sys_cl_unity = feedback(sys_open, K_unity);

% Simulate zero-input response (only initial conditions)
[Y_unity, t_unity, X_unity] = initial(sys_cl_unity, x0, t);

%% --- CASE B: State Feedback (LQR) ---
% The proper way to stabilize the system using full state feedback u = -Kx
Q = diag([100, 1, 100, 1]); % Penalize position and angle errors heavily
R = 1;                      % Penalize control effort
K_lqr = lqr(A, B, Q, R);    % Calculate optimal feedback gain

% Build Closed-Loop System with LQR: A_new = A - B*K
A_cl = A - B * K_lqr;
sys_cl_lqr = ss(A_cl, B, C, D_sys);

% Simulate zero-input response (only initial conditions)
[Y_lqr, t_lqr, X_lqr] = initial(sys_cl_lqr, x0, t);

%% --- 5. Plotting the Results ---
figure('Name', 'Zero-Input Closed-Loop Response', 'Position', [100, 100, 900, 600]);

% Plot 1: Unity Feedback (Unstable)
subplot(2, 1, 1);
plot(t_unity, X_unity(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_unity, X_unity(:,3), 'r', 'LineWidth', 1.5);
title('Case A: Unity Feedback on Angle (Remains Unstable)');
xlabel('Time (sec)');
ylabel('Amplitude');
legend('Cart Position (x)', 'Pendulum Angle (\theta)', 'Location', 'best');
grid on;

% Plot 2: LQR State Feedback (Stable)
subplot(2, 1, 2);
plot(t_lqr, X_lqr(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_lqr, X_lqr(:,3), 'r', 'LineWidth', 1.5);
title('Case B: Full State Feedback via LQR (Stable Asymptotically)');
xlabel('Time (sec)');
ylabel('Amplitude');
legend('Cart Position (x)', 'Pendulum Angle (\theta)', 'Location', 'best');
grid on;
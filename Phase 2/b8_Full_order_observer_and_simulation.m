% =========================================================================
% MATLAB Code: Full-Order Observer Design and Simulation
% =========================================================================
clear; clc; close all;

% --- 1. Physical Parameters ---
M = 2.0;    % Mass of the cart (kg)
m = 0.2;    % Mass of the pendulum (kg)
l = 0.5;    % Distance to the center of mass of the pendulum (m)
g = 9.81;   % Gravity (m/s^2)
I = 0.099;  % Moment of inertia of the pendulum (kg.m^2)

D = I*(M + m) + M*m*l^2;
a = (g * m^2 * l^2) / D;
b = -((M + m) * g) / D;
B2 = (I + m*l^2) / D;
B4 = -(m * l) / D;

% --- 2. Build Open-Loop State-Space ---
A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1;
     0 0 b 0];
B = [0; B2; 0; B4];
C = [1 0 0 0;
     0 0 1 0];
D_sys = [0; 0];

% Check Observability
if rank(obsv(A,C)) == 4
    disp('System is fully observable. Proceeding with Observer Design.');
else
    error('System is not observable!');
end

% --- 3. Observer Design (Pole Placement) ---
% We choose stable, fast poles for the observer.
% Since the system is somewhat slow, poles at -10 to -16 are sufficiently fast.
observer_poles = [-10, -12, -14, -16];

% Calculate Observer Gain Matrix L
% Note: place(A, B, P) finds K for (A-BK). 
% For observer (A-LC), we use duality: place(A', C', P)'
L = place(A', C', observer_poles)';

disp('--- Observer Gain Matrix L ---');
disp(L);
disp('Eigenvalues of (A - LC):');
disp(eig(A - L*C));

% --- 4. Simulation Setup: Plant vs. Observer ---
% Augmented System: [x; x_hat] -> 8 states
% Equations:
% x_dot = A*x + B*u
% x_hat_dot = L*C*x + (A - L*C)*x_hat + B*u
% 
% Augmented A_aug:
A_aug = [A, zeros(4,4);
         L*C, (A - L*C)];
B_aug = [B; B];
C_aug = eye(8); % Observe all actual and estimated states
D_aug = zeros(8,1);

sys_aug = ss(A_aug, B_aug, C_aug, D_aug);

% Initial Conditions (Real system vs. Observer's initial guess)
x0_real = [0.1; 0; 0.1; 0];      % Real initial state
x0_hat  = [0; 0; 0; 0];          % Observer starts guessing from zero
x0_aug  = [x0_real; x0_hat];

% Simulation time and input
t = 0:0.001:1;                   % Simulate for 1 second to see fast convergence
u = 0.5 * ones(size(t));         % Arbitrary input force (0.5 N step)

% Simulate
[Y, t_out, X] = lsim(sys_aug, u, t, x0_aug);

% Extract states for plotting
x_real = X(:, 1:4);
x_hat  = X(:, 5:8);
error  = x_real - x_hat;

% --- 5. Plotting the Results ---
figure('Name', 'Observer Performance', 'Position', [100, 100, 900, 600]);

% Plot State 2 (Cart Velocity) - Not directly measured
subplot(2,1,1);
plot(t_out, x_real(:,2), 'b', 'LineWidth', 2); hold on;
plot(t_out, x_hat(:,2), 'r--', 'LineWidth', 2);
title('Estimation of Unmeasured State: Cart Velocity (v_x)');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
legend('Real Velocity', 'Estimated Velocity', 'Location', 'best');
grid on;

% Plot State 4 (Angular Velocity) - Not directly measured
subplot(2,1,2);
plot(t_out, x_real(:,4), 'b', 'LineWidth', 2); hold on;
plot(t_out, x_hat(:,4), 'r--', 'LineWidth', 2);
title('Estimation of Unmeasured State: Angular Velocity (\omega)');
xlabel('Time (sec)');
ylabel('Angular Velocity (rad/s)');
legend('Real Angular Velocity', 'Estimated Angular Velocity', 'Location', 'best');
grid on;
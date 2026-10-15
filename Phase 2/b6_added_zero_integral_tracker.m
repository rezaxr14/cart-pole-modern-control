% =========================================================================
% Integral Tracker Design for System with Added Zero at s=0
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

% --- 2. Reduced 3D System (States: v_x, theta, omega) ---
% Since output is now velocity (added zero at s=0), position is unobservable.
Ar = [0, a, 0;
      0, 0, 1;
      0, b, 0];
Br = [B2; 0; B4];
Cr = [1, 0, 0]; % Output is v_x (cart velocity)

% --- 3. Augmented System for Integral Control ---
Aa = [Ar, zeros(3,1);
     -Cr, 0];
Ba = [Br; 0];

% Check Controllability of the Augmented System
if rank(ctrb(Aa, Ba)) == 4
    disp('Augmented system is fully controllable. Proceeding to design.');
else
    error('Augmented system is uncontrollable!');
end

% --- 4. LQR Controller Design ---
% Q heavily penalizes the integral state (4th state) to ensure strict tracking
% Also penalizes pendulum angle (2nd state) to maintain stability
Q = diag([10, 1000, 10, 5000]); 
R = 1;

Ka = lqr(Aa, Ba, Q, R);
K = Ka(1:3);      % State feedback gain
Ki = Ka(4);       % Integral gain

% --- 5. Closed-Loop Simulation Setup ---
% States for simulation: X_sim = [v_x, theta, omega, z]'
% Reference: r(t) = 1 m/s (Unit Step for Velocity)
A_cl = Aa - Ba*Ka;
B_cl = [0; 0; 0; 1]; % The reference input directly feeds the integrator state (z_dot = r - y)

% C_cl matrix to observe: [Velocity, Angle, Control Signal u]
C_cl = [1, 0, 0, 0;    % v_x
        0, 1, 0, 0;    % theta
       -Ka];           % Control effort u = -K*x_r - Ki*z
D_cl = [0; 0; 0];

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% Simulation time and input
t = 0:0.01:5;
r = ones(size(t)); % Constant reference velocity = 1 m/s

% Simulate (Assume 0 initial conditions)
[Y, t_out, X] = lsim(sys_cl, r, t);

% --- 6. Plotting Results ---
figure('Name', 'Integral Tracker Analysis', 'Position', [100, 100, 800, 700]);

% Plot 1: Cart Velocity Tracking
subplot(3,1,1);
plot(t, r, 'k--', 'LineWidth', 1.5); hold on;
plot(t_out, Y(:,1), 'b', 'LineWidth', 2);
title('Cart Velocity Tracking');
ylabel('Velocity (m/s)');
legend('Reference (1 m/s)', 'Actual Velocity', 'Location', 'best');
grid on;

% Plot 2: Pendulum Angle Regulation
subplot(3,1,2);
plot(t_out, Y(:,2), 'r', 'LineWidth', 2);
title('Pendulum Angle');
ylabel('Angle (rad)');
grid on;

% Plot 3: Control Signal (Force)
subplot(3,1,3);
plot(t_out, Y(:,3), 'g', 'LineWidth', 2);
title('Control Effort (Force)');
xlabel('Time (sec)');
ylabel('Force (N)');
grid on;
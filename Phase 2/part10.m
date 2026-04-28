% --- Part 10: Setpoint Control (Two Methods) ---

% 1. Define Target: Move Cart to 1 meter
ref = 1.0; 
C_reg = [1 0 0 0]; % We are regulating x (1st state)
D_reg = [0];

% --- Method A: Pre-Scaling (Nbar) ---
% Using the K from Part 7
poles = [-2 -3 -4 -5];
K = place(A, B, poles);

% Calculate Nbar
Nbar = -inv(C_reg * inv(A - B*K) * B)*ref;

fprintf('Method A: Nbar calculated as %.4f\n', Nbar);

% --- Method B: Integral Action (Augmented State) ---
% Augmented Matrices (5x5)
% A_aug = [A, 0; -C, 0]
A_aug = [A, zeros(4,1); -C_reg, 0];
B_aug = [B; 0];

% We need 5 poles now. Let's add a fast integrator pole.
poles_aug = [-2 -3 -4 -5 -6]; 

% Calculate Augmented K
K_aug = place(A_aug, B_aug, poles_aug);
K_feedback = K_aug(1:4); % Gains for states
K_integral = K_aug(5);   % Gain for integrator

fprintf('Method B: Integral Gain Ki = %.4f\n', K_integral);

% --- SIMULATION COMPARISON ---
t = 0:0.01:8;

% Simulate Method A (Closed Loop)
% dx/dt = (A-BK)x + B*Nbar*r
sys_A = ss(A - B*K, B*Nbar, C_reg, D_reg);
[y_a, t, x_a] = step(sys_A, t); % Step of amplitude 1

% Simulate Method B (Augmented Closed Loop)
% This requires careful construction. 
% The loop is: u = -Kx - Ki*xi
sys_B = ss(A_aug - B_aug*K_aug, [0;0;0;0;ref], [C_reg 0], 0);
[y_b, t, x_b] = step(sys_B, t);

% Plot Results
figure;
plot(t, y_a, 'b--', 'LineWidth', 2); hold on;
plot(t, y_b, 'r', 'LineWidth', 2);
yline(1.0, 'k:');
title('Part 10: Setpoint Tracking (Target x = 1m)');
legend('Method A (Nbar)', 'Method B (Integral)', 'Reference');
xlabel('Time (s)'); ylabel('Cart Position (m)');
grid on;
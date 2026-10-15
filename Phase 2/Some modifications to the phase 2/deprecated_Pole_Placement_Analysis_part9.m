% --- Part 9: Pole Placement Analysis ---

% Define 3 sets of poles
poles_slow = [-1 -1.5 -2 -2.5];
poles_mod  = [-2 -3 -4 -5];
poles_fast = [-4 -5 -6 -7];

% Calculate Gain Matrices
K_slow = place(A, B, poles_slow);
K_mod  = place(A, B, poles_mod);
K_fast = place(A, B, poles_fast);

% Display Gains to see the difference
fprintf('Gain Matrix (Slow): [%.2f %.2f %.2f %.2f]\n', K_slow);
fprintf('Gain Matrix (moderate): [%.2f %.2f %.2f %.2f]\n', K_mod);
fprintf('Gain Matrix (Fast): [%.2f %.2f %.2f %.2f]\n', K_fast);

% Simulation Setup
x0 = [0; 0; 0.1; 0]; % Initial error: 0.1 rad (approx 5.7 deg)
t = 0:0.01:5;

% Create Closed Loop Systems
sys_slow = ss(A - B*K_slow, zeros(4,1), C, D);
sys_mod  = ss(A - B*K_mod,  zeros(4,1), C, D);
sys_fast = ss(A - B*K_fast, zeros(4,1), C, D);

% Simulate Responses
[y_s, t, x_s] = initial(sys_slow, x0, t);
[y_m, t, x_m] = initial(sys_mod, x0, t);
[y_f, t, x_f] = initial(sys_fast, x0, t);

% Calculate Control Effort (Force u = -K*x) manually for plotting
u_slow = -(K_slow * x_s')';
u_mod  = -(K_mod  * x_m')';
u_fast = -(K_fast * x_f')';

% --- PLOTTING ---
figure;

% Subplot 1: System Performance (Angle)
subplot(2,1,1);
plot(t, x_s(:,3), 'g', 'LineWidth', 2); hold on;
plot(t, x_m(:,3), 'b', 'LineWidth', 2);
plot(t, x_f(:,3), 'r', 'LineWidth', 2);
title('Performance Comparison: Pole Angle');
ylabel('Angle (rad)');
legend('Slow Poles', 'Moderate Poles', 'Fast Poles');
grid on;

% Subplot 2: Control Effort (Force)
subplot(2,1,2);
plot(t, u_slow, 'g', 'LineWidth', 2); hold on;
plot(t, u_mod, 'b', 'LineWidth', 2);
plot(t, u_fast, 'r', 'LineWidth', 2);
title('Cost Comparison: Control Force Input');
xlabel('Time (s)');
ylabel('Force (N)');
legend('Low Effort', 'Moderate Effort', 'High Effort');
grid on;

disp('Comparison Complete. Notice the trade-off in the plots.');
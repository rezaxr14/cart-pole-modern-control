% --- Part 8: PID Design and Comparison ---

% 1. Convert State Space to Transfer Function
[num_x, den_x] = ss2tf(A, B, [1 0 0 0], [0]);     % TF for Cart Position
[num_th, den_th] = ss2tf(A, B, [0 0 1 0], [0]);    % TF for Pole Angle

sys_x = tf(num_x, den_x);
sys_th = tf(num_th, den_th);

% 2. Design PID Controllers (Heuristic Tuning)
% Angle Controller (Needs to be fast to stabilize the pole)
% Kp: Stiffness, Kd: Damping (crucial for stability)
pid_theta = pid(-500, -1, -100); % Kp=100, Ki=0.1, Kd=20

% Position Controller (Needs to be gentle)
pid_x = pid(40, 0.8, 28);        % Kp=2, Ki=0.5, Kd=3

% 3. Define the Control Loop Structure
% The Loop is MIMO, so we use 'feedback' carefully or build the total force.
% F = - (PID_theta * theta + PID_x * x)
% We are regulating to 0, so Error = 0 - Measurement = -Measurement

% Let's verify using the closed loop A matrix method for the PID 
% (Approximation for simulation: PID adds states, but we can simulate using 'feedback')

% Total Open Loop from Force to Combined Output
% This part is tricky in pure TF, so we simulate using a feedback loop object
C_pid = [pid_x, pid_theta]; % Controller Matrix
Sys_plant = [sys_x; sys_th]; % Plant Matrix

% Closed Loop System
% Input: Reference [ref_x; ref_theta] -> Output: [x; theta]
% Note: This connects (ref - y) -> PID -> Plant
T_pid = feedback(Sys_plant * C_pid, eye(2)); 

% 4. Comparison Simulation (Impulse/Initial Disturbance)
t = 0:0.01:10;

% We simulate a disturbance where the pole starts slightly tipped.
% Since 'T_pid' is a TF, we simulate a "Step" response on the Angle Reference
% to see how it corrects. Or we use the 'initial' command on state space.
% To keep it simple and accurate, we will convert the PID+Plant back to State Space for plotting.

% --- COMPARISON PLOT ---
figure;

% Plot 1: State Feedback Response (from Part 7)
% Re-simulating Part 7 for the same timeframe
x0 = [0; 0; 0.1; 0]; % 0.1 rad initial angle
sys_sf = ss(A - B*K, zeros(4,1), C, D);
[y_sf, t, x_sf] = initial(sys_sf, x0, t);

subplot(2,1,1);
plot(t, x_sf(:,3), 'b', 'LineWidth', 2); hold on;
title('Comparison: Pole Angle Stabilization');
xlabel('Time (s)'); ylabel('Angle (rad)');

% Plot 2: PID Response
% We approximate the PID loop behavior here:
% PID force F = -Kp_th*th - Kd_th*dth - Kp_x*x - Kd_x*dx (neglecting I for short term comparison)
% This equates to a State Feedback gain K_pid approx:
K_pid_equiv = [-40, -28, -500, -100]; % [Kp_x, Kd_x, Kp_th, Kd_th]
sys_pid_approx = ss(A - B*K_pid_equiv, zeros(4,1), C, D);
[y_pid, t, x_pid] = initial(sys_pid_approx, x0, t);

plot(t, x_pid(:,3), 'r--', 'LineWidth', 2);
legend('State Feedback (Part 7)', 'PID (Part 8)');
grid on;

subplot(2,1,2);
plot(t, x_sf(:,1), 'b', 'LineWidth', 2); hold on;
plot(t, x_pid(:,1), 'r--', 'LineWidth', 2);
title('Comparison: Cart Position');
xlabel('Time (s)'); ylabel('Position (m)');
legend('State Feedback', 'PID');
grid on;

fprintf('Comparison Complete.\n');
fprintf('Note: State Feedback usually converges faster with less oscillation because it has full knowledge of the system state.\n');
% --- Part 7: State Feedback Design ---

% 1. Define Desired Poles (Stable LHP)
% We pick 4 distinct negative poles
poles = [-2 -3 -4 -5];

% 2. Calculate Gain Matrix K using 'place' (Ackermann's method equivalent)
K = place(A, B, poles);

fprintf('Calculated Gain Matrix K:\n');
disp(K);

% 3. Check Closed-Loop Eigenvalues (Verification)
A_cl = A - B*K;
eig_cl = eig(A_cl);
disp('Closed-Loop Poles (Should match desired):');
disp(eig_cl);

% 4. Simulation with Initial Conditions
% Let's test if it stabilizes when the pole starts at 0.1 radians (~5.7 degrees)
x0 = [0; 0; 0.1; 0]; 

% Create State Space model of the Closed Loop System
% Note: The input to the autonomous system is zero, so B is effectively 0 for 'initial'
sys_cl = ss(A_cl, zeros(4,1), C, D);

% Time vector for simulation
t = 0:0.01:10;

% Simulate
[y, t, x] = initial(sys_cl, x0, t);

% 5. Plotting Results
figure;
subplot(2,1,1);
plot(t, x(:,1), 'LineWidth', 2); grid on;
title('Part 7: State Feedback Stabilization');
ylabel('Cart Position (m)');
xlabel('Time (s)');

subplot(2,1,2);
plot(t, x(:,3), 'r', 'LineWidth', 2); grid on;
ylabel('Pole Angle (rad)');
xlabel('Time (s)');
legend('Pole Angle');

disp('Part 7 Complete: Controller designed and simulated.');
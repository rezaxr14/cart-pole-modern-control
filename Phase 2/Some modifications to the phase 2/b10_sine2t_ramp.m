% =========================================================================
% Gain Calculations: Static Pre-filter vs. Augmented Integral + Disturbance
% =========================================================================
% 
% 
% % --- 1. System Parameters & Linearization ---
% M = 2.0; 
% m = 0.2; 
% l = 0.5; 
% g = 9.81; 
% I = 0.099;

% Common denominator and linearized matrix elements (around theta = 0)
D = (M + m)*(I + m*l^2) - (m*l)^2;
a23 = -(m^2 * l^2 * g) / D;
a43 = ((M + m) * m * g * l) / D;
b2  = (I + m*l^2) / D;
b4  = -(m * l) / D;

% State-Space Matrices: x = [pos, vel, angle, ang_vel]'
AA = [0 1 0 0;
     0 0 a23 0;
     0 0 0 1;
     0 0 a43 0];
BB = [0; b2; 0; b4];
CC = [1 0 0 0]; % Tracking position

%% --- 2. STATIC CONTROLLER CALCULATION ---
disp('=== STATIC CONTROLLER ===');
% LQR Weights for base plant
Q_static = diag([10, 10, 1000, 10]); 
R_static = 0.1;

% Calculate state feedback gain
k_state_static = lqr(AA, BB, Q_static, R_static);
% k_state_static = K;
% Calculate N_bar (Pre-filter gain)
% Formula ensures zero steady-state error for a STEP input
% N_bar = - [ CC * (AA - BB*K)^-1 * BB ]^-1
closed_loop_A = AA - BB*k_state_static;
nbarr = -1 / (CC * (closed_loop_A \ BB)); 
% nbarr = Nbar;
fprintf('k_state_static = [%.4f, %.4f, %.4f, %.4f]\n', k_state_static);
fprintf('nbarr = %.4f\n\n', nbarr);

%% --- 3. INTEGRAL + DISTURBANCE CONTROLLER CALCULATION ---
disp('=== INTEGRAL & DISTURBANCE CONTROLLER ===');
% To track a reference and reject sin(2t), we augment the plant.
% 1. Integrator state: z_dot = r - Cx
% 2. Disturbance states (Oscillator for sin(2t) -> w_ddot + 4w = 0)
A_osc = [0, 1; 
        -4, 0];
B_osc = [0; 1];

% Augmented State Vector: Xa = [x(4), z_int(1), w_osc(2)]' -> 7 States
Aa = [AA,              zeros(4,1), zeros(4,2);
     -CC,              0,          zeros(1,2);
     -B_osc*CC,        zeros(2,1), A_osc];
Ba = [BB; 0; 0; 0];

% Check controllability of the augmented system
if rank(ctrb(Aa, Ba)) < 7
    error('Augmented system is not fully controllable.');
end

% LQR Weights for augmented system
% Heavily penalize the integrator (state 5) and oscillator (states 6,7)
Q_int = diag([10, 1, 100, 10, 500, 1000, 1000]); 
R_int = 1;

% Calculate full augmented gain matrix
K_aug = lqr(Aa, Ba, Q_int, R_int);

% Extract individual gains
k_state_int = K_aug(1:4);       % Feedback for plant states
k_int       = K_aug(5);         % Feedback for integrator state
k_dist      = K_aug(6:7);       % Feedback for disturbance states

fprintf('k_state_int = [%.4f, %.4f, %.4f, %.4f]\n', k_state_int);
fprintf('k_int       = %.4f\n', k_int);
fprintf('k_dist      = [%.4f, %.4f]\n', k_dist(1), k_dist(2));
%% 1. System Parameters (from your fcn block)
M = 2.0; m = 0.2; l = 0.5; g = 9.81; I = 0.099;

% Denominator at equilibrium (cos(0) = 1, sin(0) = 0, dtheta = 0)
D = (M + m)*(I + m*l^2) - (m*l)^2;

%% 2. Linearized State-Space Matrices (x = [x; dx; theta; dtheta])
% Derivatives calculated at theta = 0
A = [0, 1, 0, 0;
     0, 0, -(m^2*l^2*g)/D, 0;
     0, 0, 0, 1;
     0, 0, (M+m)*m*g*l/D, 0];

B = [0; (I + m*l^2)/D; 0; -(m*l)/D];

% Output matrix C (We measure Position x and Angle theta)
C = [1, 0, 0, 0;  % Cart Position
     0, 0, 1, 0]; % Pole Angle

%% 3. Controller Design (LQR)
Q = diag([10, 1, 100, 1]); % Penalize angle error heavily
R = 0.1;
K_lqr = lqr(A, B, Q, R);

%% 4. Full-Order Luenberger Observer
% Place observer poles 5x faster than controller poles
cont_poles = eig(A - B*K_lqr);
obs_poles = 60 * cont_poles; 
L = place(A', C', obs_poles)';

%% 5. Reduced-Order Observer Design
% Partitioning: x1 = [x; theta] (Measured), x2 = [dx; dtheta] (Estimated)
% Rearrange A and B to match [x1; x2] order
A_new = [A(1,1), A(1,3), A(1,2), A(1,4);  % Row 1 & 3 (x, theta)
         A(3,1), A(3,3), A(3,2), A(3,4);
         A(2,1), A(2,3), A(2,2), A(2,4);  % Row 2 & 4 (dx, dtheta)
         A(4,1), A(4,3), A(4,2), A(4,4)];

B_new = [B(1); B(3); B(2); B(4)];

% Extract Sub-matrices
A11 = A_new(1:2, 1:2); A12 = A_new(1:2, 3:4);
A21 = A_new(3:4, 1:2); A22 = A_new(3:4, 3:4);
B1  = B_new(1:2);      B2  = B_new(3:4);

% Design L_red for the reduced system (A22, A12)
% We want to place poles for (A22 - L_red*A12)
L_red = place(A22', A12', obs_poles(1:2))';

% Pre-calculate matrices for Simulink implementation
F_mat = A22 - L_red*A12;
G_mat = B2 - L_red*B1;
H_mat = A21 - L_red*A11 + F_mat*L_red;
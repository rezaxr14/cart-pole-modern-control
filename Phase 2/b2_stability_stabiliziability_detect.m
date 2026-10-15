% =========================================================================
% MATLAB Code for Stability, Stabilizability, and Detectability Analysis
% =========================================================================

clear; clc;

% --- Define Physical Parameters ---
M = 2.0;    % Mass of the cart (kg)
m = 0.2;    % Mass of the pendulum (kg)
l = 0.5;    % Distance to the center of mass of the pendulum (m)
g = 9.81;   % Gravity (m/s^2)
I = 0.099;  % Moment of inertia of the pendulum (kg.m^2)

% --- Calculate Matrix Elements ---
D = I*(M + m) + M*m*l^2;
a = (g * m^2 * l^2) / D;
b = -((M + m) * g) / D;
B2 = (I + m*l^2) / D;
B4 = -(m * l) / D;

% --- Build State-Space Matrices ---
A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1;
     0 0 b 0];

B = [0; B2; 0; B4];

C = [1 0 0 0;
     0 0 1 0];

%% --- 1. Stability Analysis ---
disp('--- 1. Stability Analysis ---');
eigenvalues = eig(A);
disp('Eigenvalues of the system:');
disp(eigenvalues);

% Check for Asymptotic Stability
if all(real(eigenvalues) < 0)
    disp('Result: The system is ASYMPTOTICALLY STABLE (All real parts < 0).');
else
    disp('Result: The system is NOT asymptotically stable.');
end

% Check for Lyapunov Stability for repeated roots at origin
% By checking geometric multiplicity (nullity of (A - lambda*I))
lambda = 0;
nullity = size(A, 1) - rank(A - lambda*eye(size(A)));
algebraic_multiplicity = sum(eigenvalues == lambda);

if any(real(eigenvalues) > 0)
    disp('Result: The system is UNSTABLE (Has eigenvalues with positive real parts).');
elseif algebraic_multiplicity > nullity
    disp('Result: The system is UNSTABLE in Lyapunov sense.');
    disp('Reason: Repeated root at 0 has a Jordan block size > 1 (Algebraic Multiplicity > Geometric Multiplicity).');
else
    disp('Result: The system is LYAPUNOV (MARGINALLY) STABLE.');
end
disp(' ');

%% --- 2. Stabilizability (via Controllability) ---
disp('--- 2. Stabilizability & Controllability ---');
Co = ctrb(A, B);         % Controllability matrix
rank_Co = rank(Co);      % Rank of controllability matrix
n = size(A, 1);          % Number of states (4)

fprintf('Rank of Controllability Matrix: %d\n', rank_Co);
if rank_Co == n
    disp('Result: System is FULLY CONTROLLABLE.');
    disp('Conclusion: Since it is controllable, it is strictly STABILIZABLE.');
else
    disp('Result: System is NOT fully controllable.');
    disp('Need further PBH test to check stabilizability of unstable modes.');
end
disp(' ');

%% --- 3. Detectability (via Observability) ---
disp('--- 3. Detectability & Observability ---');
Ob = obsv(A, C);         % Observability matrix
rank_Ob = rank(Ob);      % Rank of observability matrix

fprintf('Rank of Observability Matrix: %d\n', rank_Ob);
if rank_Ob == n
    disp('Result: System is FULLY OBSERVABLE.');
    disp('Conclusion: Since it is observable, it is strictly DETECTABLE.');
else
    disp('Result: System is NOT fully observable.');
    disp('Need further PBH test to check detectability of unstable modes.');
end
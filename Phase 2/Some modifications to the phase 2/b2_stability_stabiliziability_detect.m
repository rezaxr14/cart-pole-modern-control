% =========================================================================
% MATLAB Code for Stability, Stabilizability, and Detectability Analysis
% =========================================================================

clear;
clc;

%% Physical Parameters
M = 2.0;      % Mass of the cart (kg)
m = 0.2;      % Mass of the pendulum (kg)
l = 0.5;      % Distance to pendulum COM (m)
g = 9.81;     % Gravity (m/s^2)
I = 0.099;    % Pendulum moment of inertia (kg.m^2)

%% Common Denominator
Delta = I*(M + m) + M*m*l^2;

%% State Matrix Coefficients
a = -(g*m^2*l^2)/Delta;
b = ((M + m)*g*m*l)/Delta;

%% Input Matrix Coefficients
B2 = I/Delta;
B4 = -m*l/Delta;

%% State-Space Matrices
A = [0 1 0 0;
     0 0 a 0;
     0 0 0 1;
     0 0 b 0];

B = [0;
     B2;
     0;
     B4];

C = [1 0 0 0;
     0 0 1 0];

%% ------------------------------------------------------------------------
%% 1. Stability Analysis
%% ------------------------------------------------------------------------
disp('------------------------------------------------------');
disp('1. Stability Analysis');
disp('------------------------------------------------------');

eigenvalues = eig(A);

disp('Eigenvalues of the system:');
disp(eigenvalues);

tol = 1e-8;

% Asymptotic Stability
if all(real(eigenvalues) < -tol)
    disp('Result: The system is ASYMPTOTICALLY STABLE.');
else
    disp('Result: The system is NOT asymptotically stable.');
end

% Lyapunov Stability
lambda = 0;

algebraic_multiplicity = sum(abs(eigenvalues - lambda) < tol);
geometric_multiplicity = size(A,1) - rank(A - lambda*eye(size(A)), tol);

if any(real(eigenvalues) > tol)
    disp('Result: The system is UNSTABLE.');
    disp('Reason: At least one eigenvalue lies in the right-half plane.');
elseif algebraic_multiplicity > geometric_multiplicity
    disp('Result: The system is UNSTABLE in the Lyapunov sense.');
    disp('Reason: A Jordan block exists for the repeated zero eigenvalue.');
elseif any(abs(real(eigenvalues)) < tol)
    disp('Result: The system is LYAPUNOV (MARGINALLY) STABLE.');
else
    disp('Result: The system is STABLE.');
end

disp(' ');

%% ------------------------------------------------------------------------
%% 2. Controllability & Stabilizability
%% ------------------------------------------------------------------------
disp('------------------------------------------------------');
disp('2. Controllability & Stabilizability');
disp('------------------------------------------------------');

n = size(A,1);

Co = ctrb(A,B);
rank_Co = rank(Co);

fprintf('Rank of Controllability Matrix = %d\n', rank_Co);

if rank_Co == n
    disp('Result: The system is FULLY CONTROLLABLE.');
    disp('Conclusion: The system is STABILIZABLE.');
else
    disp('Result: The system is NOT fully controllable.');
    disp('Conclusion: Perform the PBH test to determine stabilizability.');
end

disp(' ');

%% ------------------------------------------------------------------------
%% 3. Observability & Detectability
%% ------------------------------------------------------------------------
disp('------------------------------------------------------');
disp('3. Observability & Detectability');
disp('------------------------------------------------------');

Ob = obsv(A,C);
rank_Ob = rank(Ob);

fprintf('Rank of Observability Matrix = %d\n', rank_Ob);

if rank_Ob == n
    disp('Result: The system is FULLY OBSERVABLE.');
    disp('Conclusion: The system is DETECTABLE.');
else
    disp('Result: The system is NOT fully observable.');
    disp('Conclusion: Perform the PBH test to determine detectability.');
end
% =========================================================================
% MATLAB Code for Inverted Pendulum on a Cart
% Eigenvalues, Real Block Diagonal Form, Transfer Functions
% =========================================================================

clear;
clc;
close all;

%% Physical Parameters

M = 2.0;      % Cart mass (kg)
m = 0.2;      % Pendulum mass (kg)
l = 0.5;      % Distance to pendulum COM (m)
g = 9.81;     % Gravity (m/s^2)
I = 0.099;    % Pendulum inertia (kg.m^2)

%% Common Denominator

D = I*(M+m) + M*m*l^2;

%% State-Space Coefficients

a  = (g*m^2*l^2)/D;
b  = ((M+m)*m*g*l)/D;

B2 = (I + m*l^2)/D;
B4 = -(m*l)/D;

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

Dsys = zeros(2,1);

%% Display Matrices

disp('==============================')
disp('Matrix A')
disp(A)

disp('Matrix B')
disp(B)

disp('Matrix C')
disp(C)

%% Eigenvalues

disp('==============================')
disp('Eigenvalues')
lambda = eig(A)

%% Eigenvectors

[V,Lambda] = eig(A);

disp('==============================')
disp('Eigenvector Matrix')
disp(V)

disp('Diagonal Eigenvalue Matrix')
disp(Lambda)

%% Real Block Diagonal Form

[T,J] = cdf2rdf(V,Lambda);

disp('==============================')
disp('Real Block Diagonal Matrix (J)')
disp(J)

disp('Transformation Matrix (T)')
disp(T)

%% Verify Similarity Transformation

disp('==============================')
disp('Verification: inv(T)*A*T')

disp(inv(T)*A*T)

%% State-Space Model

sys = ss(A,B,C,Dsys);

sys.InputName = {'Force'};
sys.OutputName = {'Cart Position','Pendulum Angle'};

%% Transfer Functions

disp('==============================')
disp('Transfer Functions')

G = tf(sys)

%% Poles

disp('==============================')
disp('Poles')

pole(sys)

%% Zeros

disp('==============================')
disp('Transmission Zeros')

tzero(sys)

%% Pole-Zero Map

figure
pzmap(sys)
grid on
title('Pole-Zero Map')

%% Step Response

figure
step(sys)
grid on
title('Step Response')

%% Impulse Response

figure
impulse(sys)
grid on
title('Impulse Response')

%% Bode Plot

figure
bode(sys)
grid on
title('Bode Diagram')

%% Root Locus

figure
rlocus(sys(2))
grid on
title('Root Locus (Pendulum Angle)')

%% Controllability

Co = ctrb(A,B);

disp('==============================')
disp('Rank of Controllability Matrix')

rank(Co)

%% Observability

Ob = obsv(A,C);

disp('==============================')
disp('Rank of Observability Matrix')

rank(Ob)

%% End

disp('==============================')
disp('Analysis Completed Successfully')
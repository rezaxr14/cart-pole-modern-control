% =========================================================================
% MATLAB Code for Inverted Pendulum on a Cart
% Part 1: Real Block Diagonal Form & Transformation Matrix
% Part 2: Transfer Functions & Pole-Zero Map
% =========================================================================

clear; clc; close all;

% --- Define Physical Parameters ---
  
M = 2.0;    % Mass of the cart (kg)
m = 0.2;    % Mass of the pendulum (kg)
l = 0.5;    % Distance to the center of mass of the pendulum (m)
g = 9.81;   % Gravity (m/s^2)
I = 0.099;  % Moment of inertia of the pendulum (kg.m^2)

% --- Calculate Matrix Elements ---
% Common denominator term
D = I*(M + m) + M*m*l^2;

% Simplified matrix coefficients
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

D_sys = [0; 0];

%% --- Part 1: Eigenvalues, J Matrix, and T Matrix ---

% Find complex eigenvalues and complex eigenvectors
[T_complex, J_complex] = eig(A);

% Convert complex diagonal form to Real Block Diagonal Form
[T, J] = cdf2rdf(T_complex, J_complex);

disp('==================================================');
disp('Real Block Diagonal Form (J):');
disp(J);
disp('Transformation Matrix (T):');
disp(T);
disp('==================================================');

%% --- Part 2: Transfer Functions and Pole-Zero Map ---

% Create State-Space model object in MATLAB
sys = ss(A, B, C, D_sys);
sys.OutputName = {'Cart Position (X)', 'Pendulum Angle (Theta)'};
sys.InputName = {'Force (F)'};

% Convert State-Space to Transfer Function
sys_tf = tf(sys);

disp('Transfer Functions of the System:');
sys_tf

% Plot Pole-Zero Map for both outputs
figure;
pzmap(sys);
title('Pole-Zero Map of the System (X and Theta)');
grid on;

% Make the plot visually nicer (adjusting limits)
axis([-1 1 -10 10]);
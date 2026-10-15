% =========================================================================
% MATLAB Code for BIBO (Input-Output) Stability Analysis
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

Dsys = zeros(2,1);

%% Create State-Space Model
sys = ss(A,B,C,Dsys);

%% Transfer Functions
disp('------------------------------------------------------');
disp('Transfer Functions');
disp('------------------------------------------------------');

sys_tf = tf(sys)

%% BIBO Stability Analysis
disp(' ');
disp('------------------------------------------------------');
disp('BIBO Stability Analysis');
disp('------------------------------------------------------');

% Obtain poles
sys_poles = pole(sys);

disp('Poles of the system:');
disp(sys_poles);

tol = 1e-8;

if all(real(sys_poles) < -tol)
    disp('Result: The system is BIBO STABLE.');
    disp('Conclusion: All poles lie in the left-half complex plane.');
else
    disp('Result: The system is NOT BIBO STABLE.');

    if any(real(sys_poles) > tol)
        disp('Reason: At least one pole lies in the right-half plane.');
    else
        disp('Reason: One or more poles lie on the imaginary axis.');
    end
end
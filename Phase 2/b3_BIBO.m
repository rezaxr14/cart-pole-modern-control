% =========================================================================
% MATLAB Code for BIBO (Input-Output) Stability Analysis
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
 
D_sys = [0; 0];

% --- Create System and Transfer Functions ---
sys = ss(A, B, C, D_sys);
sys_tf = tf(sys);

disp('--- Transfer Functions ---');
sys_tf

%% --- BIBO Stability Check ---
disp('--- BIBO Stability Analysis ---');

% Get the poles of the system's transfer functions
sys_poles = pole(sys_tf);

disp('Poles of the Transfer Function:');
disp(sys_poles);

% Check BIBO condition: All poles must have strictly negative real parts
% We use a small tolerance to handle MATLAB's floating-point precision for values very close to zero
tolerance = 1e-10; 

is_BIBO_stable = true;
for i = 1:length(sys_poles)
    if real(sys_poles(i)) >= -tolerance
        is_BIBO_stable = false;
        break;
    end
end

if is_BIBO_stable
    disp('Result: The system is BIBO STABLE.');
    disp('Conclusion: All poles have strictly negative real parts.');
else
    disp('Result: The system is NOT BIBO STABLE.');
    disp('Conclusion: There is at least one pole with a real part >= 0 (e.g., on the imaginary axis or right-half plane).');
end
clc; clear;

% System Parameters
M = 2.0; m = 0.2; l = 0.5; g = 9.81; I = 0.099;
D_term = I*(M+m) + M*m*l^2;

A = [0, 1, 0, 0;
     0, 0, -(g*m^2*l^2)/D_term, 0;
     0, 0, 0, 1;
     0, 0, (g*(M+m))/D_term, 0];

B = [0; (I + m*l^2)/D_term; 0; -(m*l)/D_term];
C = [1, 0, 0, 0; 0, 0, 1, 0];
D = [0; 0];

% Angle Controller (Stabilizer)
Kp_theta = -500; 
Ki_theta = -1; 
Kd_theta = -100;

% Position Controller (Tracker)
Kp_x = -10; 
Ki_x = -0.5; 
Kd_x = -10;

% Initial Condition for Test
x0 = [0; 0; 0.1; 0]; % Starts tilted by 0.1 rad
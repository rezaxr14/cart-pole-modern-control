function [ddx, ddtheta] = fcn(F, theta, dtheta)
    % Non-Linear Cart-Pole Dynamics
    % Inputs: F (Force), theta (Angle), dtheta (Angular Velocity)
    % Outputs: ddx (Cart Acceleration), ddtheta (Angular Acceleration)
    
    % Parameters (Make sure these match your LQR design script!)
    M = 2.0;    % Mass of the cart (kg)
    m = 0.2;    % Mass of the pendulum (kg)
    l = 0.5;    % Distance to the center of mass of the pendulum (m)
    g = 9.81;   % Gravity (m/s^2)
    I = 0.099;  % Moment of inertia of the pendulum (kg.m^2)
    
    % Trig functions
    s = sin(theta);
    c = cos(theta);
    
    % Common Denominator
    D = (M + m)*(I + m*l^2) - (m*l*c)^2;
    
    % Cart Acceleration (ddx)
    num_x = (I + m*l^2)*(F + m*l*dtheta^2*s) - (m*l*c)*(m*g*l*s);
    ddx = num_x / D;
    
    % Angular Acceleration (ddtheta)
    num_theta = (M + m)*(m*g*l*s) - (m*l*c)*(F + m*l*dtheta^2*s);
    ddtheta = num_theta / D;
end
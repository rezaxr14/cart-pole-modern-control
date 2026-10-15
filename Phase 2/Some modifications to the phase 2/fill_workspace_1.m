clear;
clc;
% System Parameters
M = 2.0; m = 0.2; l = 0.5; g = 9.81; I = 0.099;

% Derived Denominator
Den = (M + m)*(I + m*l^2) - (m*l)^2;

% Linearized Matrix Elements
A23 = -(m^2 * l^2 * g) / Den;
A43 = ((M + m) * m * g * l) / Den;
B2  = (I + m*l^2) / Den;
B4  = -(m*l) / Den;

% State Space Matrices
A = [0 1 0 0;
     0 0 A23 0;
     0 0 0 1;
     0 0 A43 0];

B = [0; B2; 0; B4];

% Output: x and theta
C = [1 0 0 0;
     0 0 1 0];

D = [0; 0];

% Calculate Eigenvalues
eigenvals = eig(A);
disp('Eigenvalues are'); disp(eigenvals);

% Part 3 Analysis
Cm = ctrb(A, B);
Om = obsv(A, C);

rank_Cm = rank(Cm);
rank_Om = rank(Om);

fprintf('Controllability Rank: %d (Full Rank = 4)\n', rank_Cm);
if rank_Cm == 4, disp('System is Controllable'); else, disp('Uncontrollable'); end

fprintf('Observability Rank: %d (Full Rank = 4)\n', rank_Om);
if rank_Om == 4, disp('System is Observable'); else, disp('Unobservable'); end

disp(" ");
disp("------PBH TEST------");
disp(" ");


function Ctrb_CheckResults = PB_Check(lambda, A, B, C)
    n = length(lambda);
    
    % Initialize result array
    Ctrb_CheckResults = zeros(n, 1);
    
    for i = 1:n
        lambda_current = lambda(i);
        
        disp("PBH Test for controllability lambda = ");disp(lambda_current);
        controllability_PBH = [eye(4) * lambda_current - A B]
        disp("rank= ");disp(rank(controllability_PBH))
        disp("rref= ");disp(rref(controllability_PBH))
        disp("PBH Test for observability lambda = ");disp(lambda_current);
        observability_PBH = [eye(4) * lambda_current - A; C]
        disp("rank= ");disp(rank(observability_PBH))
        disp("rref= ");disp(rref(observability_PBH))
    end
end

Ctrb_CheckResults = PB_Check(eigenvals, A, B, C);


% contains the lqr controller and loads resuling K into 
% the workspace for use in simulink non-linear model

linear_matrices; % contains all the constants and the linearised state space matrices

% for reference, states: x = [phi; theta; phi_dot; theta_dot]^T

olp = eig(linA); %open loop poles

% the initial most values that worked well, use these as benchmark
% Q = diag([1e2, 5e-1, 1e1, 1e0]);
% R = 5e1;

% the value K from this script is then used 
% in the Python code in controller() function.
% to check how good this controller is,
% K is used in the simulink model.
Q = diag([5e-3, 9e1, 5e-6, 3e-1]);
R = 3e0;
K = lqr(linA,linB,Q,R) 
%% Reference double cannonical double integrator example from: http://sam.pfrommer.us/tutorial-single-shooting-trajectory-optimization-with-matlab
clear;clc;close all;
%% Initialization
x0 = zeros(76,6); % Initial guess for all the decision variables at all times
%Lower and upper bounds for simulation
lb = []; % No bounds on torque
ub = [];

% Define start and stop times, set an h to keep outputs at constant time
% interval. You should only change h for different requirements.
t = 0;
tfinal = 1.5;
h = 0.02; % seconds
tspan = t:h:tfinal;
N = size(tspan,2);
decVar = 6;
x0 = zeros(N,decVar);
%%
% Initialize state and contact mode
x0(1,1) = -pi/2; %Changing the initial matrix with starting position of the links
x0(end,1) = pi/2;
%%
% Create options that choose iterative display and the interior-point algorithm
options = optimoptions(@fmincon,...
    'Display','iter','Algorithm','interior-point','MaxFunctionEvaluations', 1e5);
% Run the fmincon solver with the options structure
% reporting both the location x of the minimizer and the value fval attained by the objective function
q = fmincon(@objfcn,x0,[],[],[],[],[],[],@contraints,options)

%% 1.3
options = optimoptions(@fmincon,...
    'Display','iter','Algorithm','interior-point','MaxFunctionEvaluations', 1e5);
% Run the fmincon solver with the options structure
% reporting both the location x of the minimizer and the value fval attained by the objective function
lb = zeros(76,6);
lb(1:76,1:2) = -3*pi/4;
lb(1:76,3:6) = -inf; 
ub = zeros(76,6);
ub(1:76,1:2) = 3*pi/4;
ub(1:76,3:6) = inf; 

q3 = fmincon(@objfcn,x0,[],[],[],[],lb,ub,@contraints,options)
animateHW11(q,h*5)
animateHW11(q3,h*10)

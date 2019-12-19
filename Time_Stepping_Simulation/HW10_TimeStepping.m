clear;clc;close all;
%% Initialization

% Define start and stop times, set an h to keep outputs at constant time
% interval. You should only change h for different requirements.
t = 0;
tfinal = 3;
h = 0.01; % seconds
tspan = t:h:tfinal;

% Initialize state and contact mode
q = [.2;1];
dq = [0;0];

% Initialize arrays for logging data
xout = []; % collection of state vectors at each time
lambdaout = []; % collection of contact forces at each time

% Initialize contact mode
oldContactMode = 0;
disp(['Initialize in mode {' '} with h value of ',num2str(h),'s.'])
%% Main Loop
syms x y x_dot y_dot dq_i1_x dq_i1_y lam_i1_x lam_i1_y q_i1_x q_i1_y m g real
m = 1;
g = 9.8;
N = [0;m*g];
C = 0;
M = m*eye(2);
q_i = q;
dq_i = dq; 
q_i1 = [q_i1_x; q_i1_y];
dq_i1 = [dq_i1_x ;dq_i1_y];
lam_i1 = [lam_i1_x ;lam_i1_y];
v = [];
for i =  1:length(tspan)
    a = [(2*q_i1_y)-q_i1_x ;(2*q_i1_y)+q_i1_x] ;
    % Solve for new state and contact forces
    % Your code here
    eq1 = (M*(dq_i1-dq_i)) + (h*(N)) + (A_vel(q_i(1,1),q_i(2,1))'*[lam_i1_x;lam_i1_y]) == 0;
    eq2 = q_i1 == q_i + (h*(dq_i1));
    eq3 = a >= 0;
    eq4 = [lam_i1_x ;lam_i1_y] <= 0;
    eq5 = a(1,1)'*lam_i1_x == 0;
    eq6 = a(2,1)'*lam_i1_y == 0;
    eqs = [eq1;eq2;eq3;eq4;eq5;eq6];
    soln = solve(eqs,[lam_i1_x ,lam_i1_y,q_i1_x, q_i1_y,dq_i1_x ,dq_i1_y],'ReturnConditions', true);
    q_i = [double(soln.q_i1_x); double(soln.q_i1_y)];
    dq_i = [double(soln.dq_i1_x); double(soln.dq_i1_y)];
    lam_i_x =double(soln.lam_i1_x);
    lam_i_y = double(soln.lam_i1_y);
    if lam_i_x <0 || lam_i_y <0
        a = [(2*q_i(2,1)-q_i(1,1)) ;(2*q_i(2,1))+q_i(1,1)];
        if a(1,1) < 10^(-4) && abs(a(2,1) >10^(-2))
            newContact = 1;
            if newContact~=oldContactMode 
                disp(['Transition from mode {', num2str(oldContactMode), '} to mode {', num2str(newContact), '} at t = ', num2str(tspan(i)), 's.']);
                oldContactMode = newContact;
            end
        end
        if a(2,1) < 10^(-4) && abs(a(1,1) >10^(-2))
            newContact = 2;
                
            if newContact~=oldContactMode
                
                disp(['Transition from mode {', num2str(oldContactMode), '} to mode {', num2str(newContact), '} at t = ', num2str(tspan(i)), 's.']);
                oldContactMode = newContact;
            end
        end
        
    end
     % Check new contact mode and determine if there has been a transition
%     % Your code here, and display the following message when appropriate
%     % Log state and contact forces
%     % Your code here
    xout = [xout;q_i' ,dq_i'];
    lambdaout = [lambdaout; lam_i_x lam_i_y];
end
disp(['Terminate in mode {', num2str(1),' ',num2str(2), '} at t = ', num2str(tfinal), 's.']);

% This function shows animation for the simulation, don't modify it
animateHW10(xout, h);
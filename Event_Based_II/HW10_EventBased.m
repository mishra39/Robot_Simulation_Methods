clear;clc;close all;
%% Initialization

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart =0;
tfinal = 3;
dt = 0.01;

% Initialize state and contact mode, you should only change q0 and
% contactMode for different requirements.
q0 = [0.2;1];
dq0 = [0;0];
x0 = [q0;dq0];
contactMode = [];
disp(['Initial condition: [',num2str(q0'),']''.'])

%% Main Loop
% Tell ode45 what event function to use and set max step size to make sure
% we don't miss a zero crossing
% Your code here
options = odeset('Events', @(t,q) contEventsFcn(t,q,contactMode) ,'MaxStep',0.07,'OutputSel' , 1,'Refine',1);

%%
% Initialize output arrays
tout = tstart;
xout = x0.'; % collection of state vectors at each time
teout = []; % collection of ode45 output
xeout = []; % collection of ode45 output
yeout = [];
ieout = []; % collection of ode45 output

%Contact Modes
J = {[],[1],[2],[3],[1,3]};
%%
while tstart < tfinal
    % Initialize simulation time vector
    tspan = [tstart 5];
    
    % Simulate
    
    [t,q,te,ye,ie] = ode45(@(t,q) odefun(t,q,contactMode),tspan,x0,options);
    % Sometimes the events function will record a nonterminal event if the
    % initial condition is a zero. We want to ignore this, so we will only
    % use the last row in the terminal state, time, and index.
    
    % Log output
    nt = length(t);
    tout = [tout; t(2:nt)];
    xout = [xout; q(2:nt,:)];
    teout = [teout; te];    % collection of ode45 output
    yeout = [yeout; ye];    % collection of ode45 output
    ieout = [ieout; ie];    % collection of ode45 output
    q_minus = q(nt,:)'; %Extracting the last row of state q
    
    disp(['Event triggered for {', num2str(contactMode), '} at time t = ', num2str(t(nt)), ' s.'])
    % Quit if simulation completes
    
%     a_pos_fun =  a_pos(q_minus(1,1),q_minus(2,1));
%     a_pos_fun_1 = a_pos_fun(1,:);
%     a_pos_fun_2 = a_pos_fun(2,:);
%     if a_pos_fun_1 <10^(-4) && a_pos_fun_2 <10^(-4)
%         ie = [];
%     end
    
    if isempty(ie) 
        disp('Final time reached');
        
        break; % abort if simulation has completed
    end
%     
    %Compute the proper contact mode via complemetarity
    new_contact = check_IV_func(t,q_minus,contactMode);
    contactMode = new_contact;
    
    % Your code here, and display the following message when appropriate
    disp(['Transition to contact mode {', num2str(contactMode), '} at time t = ', num2str(t(nt)), ' s.'])
    
    %Reset map from the old contact mode to new one
    if (ie)
        q_dot_plus = q_dot_plus_calc(q_minus,contactMode); 
        x0 = [q_minus(1,1);q_minus(2,1);q_dot_plus];
        
    else
%         If no contact, then update using ode output
        x0= q_minus;
    end

 % Update initial conditions for next iteration
     tstart = t(nt);
    options = odeset('Events', @(t,q) contEventsFcn(t,q,contactMode) ,'MaxStep',0.07,'OutputSel' , 1,'Refine',1);


%     % Stop if the particle comes to a rest
    if abs(x0(3,1)) < 10^(-4) && abs(x0(4,1)) < 10^(-4)
        disp('The particle has come to a rest')
        break;
    end 
    
end
% This function shows animation for the simulation, don't modify it
animateHW10(xout, dt);
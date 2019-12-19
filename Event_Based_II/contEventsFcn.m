function [value,isterminal,direction] = contEventsFcn(t,q,contactMode)
J = [1,2,3];
syms x y x_dot y_dot m g real
m = 1;
g = 9.8;
M = m*eye(2);
N = [0;m*g];
C = 0;

if (isempty(contactMode))       %If the particle is in unconstrained mode, cycle through all the contact modes to check IV and FA conditions
        a_pos_fun =  a_pos(q(1,1),q(2,1));
        a_pos_fun_1 = a_pos_fun(1,:);
        a_pos_fun_2 = a_pos_fun(2,:);
        q_dot = [q(3,1);q(4,1)];
        value = [a_pos_fun_1;a_pos_fun_2];
        isterminal = [1;1];  % Halt integration 
        direction = [-1;-1];   % The zero can be approached from top
end

if (contactMode == 1)
    a_pos_fun =  a_pos(q(1,1),q(2,1));
    a_pos_fun_1 = a_pos_fun(1,:);
    a_pos_fun_2 = a_pos_fun(2,:);
    
    value = [a_pos_fun_2];
    isterminal = [1];  % Halt integration 
    direction = [-1];   % The zero can be approached from top

end

if (contactMode == 2)
    a_pos_fun =   a_pos(q(1,1),q(2,1));
    a_pos_fun_1 = a_pos_fun(1,:);
    a_pos_fun_2 = a_pos_fun(2,:);    
    value = a_pos_fun_1;
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end
end
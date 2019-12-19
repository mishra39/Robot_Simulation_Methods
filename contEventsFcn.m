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
        a_pos_fun_3= a_pos_fun(3,:);
        q_dot = [q(3,1);q(4,1)];
        value = [a_pos_fun_1;a_pos_fun_2;a_pos_fun_3];
        isterminal = [1;1;1];  % Halt integration 
        direction = [-1;-1;-1];   % The zero can be approached from top
end

if (contactMode == 1)
    a_pos_fun =  a_pos(q(1,1),q(2,1));
    a_pos_fun_1 = a_pos_fun(1,:);
    a_pos_fun_2 = a_pos_fun(2,:);
    a_pos_fun_3 = a_pos_fun(3,:);
    
    value = [a_pos_fun_2;a_pos_fun_3];
    isterminal = [1;1];  % Halt integration 
    direction = [-1;-1];   % The zero can be approached from top

end

if (contactMode == 2)
    a_pos_fun =   a_pos(q(1,1),q(2,1));
    a_pos_fun_1 = a_pos_fun(1,:);
    a_pos_fun_2 = a_pos_fun(2,:);
    a_pos_fun_3 = a_pos_fun(3,:);
    
    value = a_pos_fun_1;
    isterminal = 1;  % Halt integration 
    direction = 0;   % The zero can be approached from either direction
end

if (contactMode == 3)
   
    a_pos_fun =   a_pos(q(1,1),q(2,1));
    a_pos_fun_1 = a_pos_fun(1,:);
    a_pos_fun_2 = a_pos_fun(2,:);
    a_pos_fun_3 = a_pos_fun(3,:);
    A_vel_fun_3 = [(2*q(1,1))-4, (2*q(2,1))-2];
    A_acc_fun_3 = [2, 2];
    
    q_dot = [q(3,1);q(4,1)];
    q_dot_minus = q_dot;
    mat_blk = [M A_vel_fun_3';A_vel_fun_3 0];
    mat_2 = [M*q_dot_minus;0];
    inv_mat = inv(mat_blk);
    A_dag = inv_mat(3:end,1:2);
    LAMBDA = inv_mat(3,3);
    lambda_val = (A_dag*(-N))-(A_acc_fun_3*q_dot);
    
    iv_para = inv_mat*mat_2;
    q_dot_plus = iv_para(1:2,1);
    check_val = A_vel_fun_3*q_dot_plus;
    
    value = [check_val; lambda_val];
    isterminal = [1;1]; % Halt integration 
    direction = [0;0];   % The zero can be approached from either direction
end
end
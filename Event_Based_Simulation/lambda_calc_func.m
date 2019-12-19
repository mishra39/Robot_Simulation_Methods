function FA_check =  lambda_calc_func(t,q,contactMode)
syms x y x_dot y_dot m g real
N = [0;m*g];
C = 0;
M = m*eye(2);
q_dot = q(3:4);
A_vel_fun = A_vel(q(1,1),q(2,1));
A_acc_fun = A_acc(q(3,1),q(4,1));
A_vel_fun = A_vel_fun(contactMode,:);
A_acc_fun = A_acc_fun(contactMode,:);

mat_blk = [M A_vel_fun';A_vel_fun 0];
mat_2 = [-N;-A_acc_fun*q_dot];
q_ddot = inv(mat_blk)*mat_2;
lambda_val = q_ddot(4,1);
if lambda_val > 10^(-6)
    FA_check = 1;
else
    FA_check = 0;
end


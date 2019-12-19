function q_dot_plus = q_dot_plus_calc(q,contactMode)
syms x y x_dot y_dot m g real
m = 1;
g = 9.8;
N = [0;m*g];
C = 0;
M = m*eye(2);
q_dot_minus = [q(3,1);q(4,1)];
N = [0;m*g];
A_vel_fun = A_vel(q(1,1),q(2,1));
A_acc_fun = A_acc();

if isempty(contactMode)
      q_dot_plus = q_dot_minus;    
else
    A_vel_fun = A_vel(q(1,1),q(2,1));
    A_vel_fun = A_vel_fun(contactMode,:);
    A_acc_fun = A_acc();
    A_acc_fun = A_acc_fun(contactMode,:);
    mat_blk = [M A_vel_fun';A_vel_fun 0];
    mat_2 = [-N;-A_acc_fun*q_dot_minus];
    inv_mat = inv(mat_blk);
    M_dag = inv_mat(1:2,1:2);
    A_dag = inv_mat(3,1:2);
    q_dot_plus = [q_dot_minus-((A_dag'*A_vel_fun)*q_dot_minus)];
end

end


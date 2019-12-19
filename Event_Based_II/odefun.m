function dqdt = odefun(t,q,contactMode)
syms x y x_dot y_dot m g real
m = 1;
g = 9.8;
N = [0;m*g];
C = 0;

M = m*eye(2);
q_dot = q(3:4);

if (isempty(contactMode))
    M_dag = inv(M);
    dqdt(1,1) = q(3,1);
    dqdt(2,1) = q(4,1);
    dqdt(3:4,1) = [M_dag*(-N)];
    
    
else

    A_vel_fun = A_vel(q(1,1),q(2,1));
    A_vel_fun = A_vel_fun(contactMode,:);
    A_acc_fun = A_acc();
    A_acc_fun = A_acc_fun(contactMode,:);
    mat_blk = [M A_vel_fun';A_vel_fun 0];
    mat_2 = [-N;-A_acc_fun*q_dot];
    inv_mat = inv(mat_blk);
    M_dag = inv_mat(1:2,1:2)';
    A_dag = inv_mat(3,1:2);

    dqdt(1,1) = q(3,1);
    dqdt(2,1) = q(4,1);
    dqdt(3:4,1) = [(M_dag*(-N))-((A_dag'*A_acc_fun)*dqdt(1:2,1))];
end
end
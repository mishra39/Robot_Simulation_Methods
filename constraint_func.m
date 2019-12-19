clc
syms x y x_dot y_dot m g real
q = [x;y;x_dot; y_dot];
a = [y;x+y+1;((x-2)^2)+((y-1)^2)-2];

matlabFunction(a,'File','a_pos','Comments','Position Constraints')

A = jacobian(a,q(1:2));

matlabFunction(A,'File','A_vel','Comments','Velocity Constraints')
A_vel = matlabFunction(A);
A_dot = [0 0; 0 0; (2+(x_dot*0)) (2+(0*y_dot))];
matlabFunction(A_dot,'File','A_acc','Comments','Position Constraints')

m = 1;
g = 9.8;

N = [0;m*g];
C = 0;
M = m*eye(2);

A_vel_fun = A_vel(q(1,1),q(2,1));
A_acc_fun = A_acc();
A_vel_fun = A_vel_fun(contactMode,:);
A_acc_fun = A_acc_fun(contactMode,:);

mat_blk = [M A_vel_fun';A_vel_fun 0];
mat_2 = [-N;-A_acc_fun*[q(3,1);q(4,1)]];
q_ddot = inv(mat_blk)*mat_2;
lambda_val = q_ddot(3,1);
matlabFunction(lambda_val,'File','lambdaFA','Comments','FA Constraints')
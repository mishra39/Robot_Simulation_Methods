clc
syms x y x_dot y_dot m g real
q = [x;y;x_dot; y_dot];
a = [(2*y)-x;(2*y)+x];
matlabFunction(a,'File','a_pos','Comments','Position Constraints')
A = jacobian(a,q(1:2));
matlabFunction(A,'File','A_vel','Comments','Velocity Constraints')
A_vel = matlabFunction(A);
A_dot = [0 0; 0 0; (2+(x_dot*0)) (2+(0*y_dot))];
matlabFunction(A_dot,'File','A_acc','Comments','Position Constraints')
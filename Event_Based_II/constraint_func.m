clc
syms x y x_dot y_dot m g real
q = [x;y;x_dot; y_dot];
a = [(2*y)-x;(2*y)+x];

matlabFunction(a,'File','a_pos','Comments','Position Constraints')
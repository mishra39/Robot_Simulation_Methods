function a = a_pos(x,y)
%A_POS
%    A = A_POS(X,Y)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    15-Nov-2019 01:57:55

%Position Constraints
t2 = y.*2.0;
a = [t2-x;t2+x];

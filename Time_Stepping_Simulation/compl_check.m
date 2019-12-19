function contMode_new = compl_check(q_i1,oldcontactMode)
J = [1,2];
% q_dot_minus = [q_i(3,1);q_i(4,1)];
m = 1;
g = 9.8;
N = [0;m*g];
C = 0;

M = m*eye(2);
if (isempty(oldcontactMode))       %If the particle is in unconstrained mode, cycle through all the contact modes to check IV and FA conditions
        for i = 1:size(J,2)
              a_pos_fun =  a_pos(q_i1(1,1),q_i1(2,1))
              a_pos_fun_1 = a_pos_fun(1,:);
              a_pos_fun_2 = a_pos_fun(2,:);              
              if abs(a_pos_fun(i,1)<10^(-4))
                      disp('Mode Found for empty: ')
                      disp(i)
                      contMode_new = i;
                  else
                      contMode_new = oldcontactMode;
              end
              end
else
        for i = 1:size(J,2)
            if i~=oldcontactMode
              a_pos_fun =  a_pos(q_i1(1,1),q_i1(2,1));
              a_pos_fun_1 = a_pos_fun(1,:);
              a_pos_fun_2 = a_pos_fun(2,:); 
              if abs(a_pos_fun(i,1))<10^(-4)
                  disp('Mode Found')
                  disp(i)
                      contMode_new = i;
                  else
                      contMode_new = oldcontactMode;
              end
              end
            end
end
end
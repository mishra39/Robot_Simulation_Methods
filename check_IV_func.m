function contMode_new = check_IV_func(t,q_minus,contactMode)
J = [1,2,3];
q_dot_minus = [q_minus(3,1);q_minus(4,1)];
m = 1;
g = 9.8;
N = [0;m*g];
C = 0;

M = m*eye(2);

if (isempty(contactMode))       %If the particle is in unconstrained mode, cycle through all the contact modes to check IV and FA conditions
        a_pos_fun =  a_pos(q_minus(1,1),q_minus(2,1));
        for i = 1:size(J,2)
              a_pos_fun =  a_pos(q_minus(1,1),q_minus(2,1));
              a_pos_fun_1 = a_pos_fun(1,:);
              a_pos_fun_2 = a_pos_fun(2,:);
              a_pos_fun_3= a_pos_fun(3,:);
              
              
              A_vel_fun = A_vel(q_minus(1,1),q_minus(2,1));
              A_vel_fun =  A_vel_fun(i,:);
              mat_blk = [M A_vel_fun';A_vel_fun 0];
              mat_2 = [M*q_dot_minus;0];
              iv_para = inv(mat_blk)*mat_2;
              q_dot_plus = iv_para(1:2,1);
              check_val = A_vel_fun*q_dot_plus;
              p_hat = iv_para(3,1);
              if  abs(a_pos_fun(i,1)) <=10^(-6) && check_val>=-10^(-6) && p_hat <= 10^(-6)
                  contMode_new = i;
                  
                   break;                
            else
                contMode_new = contactMode;
            end
              check_val = A_vel_fun*q_dot_plus;
        end
else
    if contactMode == 3
        contMode_new = [];
          
    else
        
        for i = 1:size(J,2)
            if i~=contactMode
                 a_pos_fun =  a_pos(q_minus(1,1),q_minus(2,1));
                  a_pos_fun_1 = a_pos_fun(1,:);
                  a_pos_fun_2 = a_pos_fun(2,:);
                  a_pos_fun_3= a_pos_fun(3,:);


                  A_vel_fun = A_vel(q_minus(1,1),q_minus(2,1));
                  A_vel_fun =  A_vel_fun(i,:);
                  mat_blk = [M A_vel_fun';A_vel_fun 0];
                  mat_2 = [M*q_dot_minus;0];
                  iv_para = inv(mat_blk)*mat_2;
                  q_dot_plus = iv_para(1:2,1);
                  check_val = A_vel_fun*q_dot_plus;
                   p_hat = iv_para(3,1);
                  if  abs(a_pos_fun(i,1)) <=10^(-6) && check_val>=-10^(-6) && p_hat <= 10^(-6)
                      contMode_new = i;
                       
                       break;                
                else
                    contMode_new = contactMode;
                end
            end
        end
    end
end
end
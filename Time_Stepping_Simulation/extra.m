 
    cont_chk = compl_check(q_i,oldContactMode);
    if cont_chk == 404
        syms lam_i1_x lam_i1_y real
        q_i1 = [0;0];
        eq1 = (M*(dq_i1-dq_i)) + (h*(N)) + (A_vel(q_i(1,1),q_i(2,1))'*[lam_i1_x;lam_i1_y]) == 0;
        eq2 = q_i1 == q_i + (h*(dq_i1));
        eqs = [eq1,eq2];
        soln = solve(eqs,[lam_i1_x, lam_i1_y,dq_i1_x ,dq_i1_y]);
        lam_i1 = [double(soln.lam_i1_x); double(soln.lam_i1_y)];
        dq_i = [double(soln.dq_i1_x); double(soln.dq_i1_y)];
        cont_chk = compl_check(q_i,oldContactMode);
   
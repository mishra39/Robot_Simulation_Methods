function [c,ceq] = contraints(x)
% This function describes all the inequality and equality constraints for
% the fmincon commnad
h = 0.02;
dq1 = x(:,3);
dq2 = x(:,4);
ddq = [];%zeros(size(q1,1)*2,0);
dq =[];% zeros(size(q1,1)*2,0);

for i=1:size(x,1)
    [M,C,N,Y] = computeDynamicMatrices([x(i,1); x(i,2)],[x(i,3); x(i,4)],[x(i,5);x(i,6)]);
    ddq_k = (inv(M))*(Y-N-(C*[x(i,3); x(i,4)]));
    ddq(i,:)=ddq_k';
end

ceq_dq1 = []; ceq_dq2 = [];ceq_q1 = [];ceq_q2 = [];
for i=1:size(x,1)-1
    ceq_dq1(i,1) = x(i+1,3)-(x(i,3))-((h/2)*(ddq(i+1,1)+ddq(i,1))); 
    ceq_dq2(i,1) = x(i+1,4)-(x(i,4))-((h/2)*(ddq(i+1,2)+ddq(i,2)));
    ceq_q1(i,1) = x(i+1,1)-x(i,1)-((h/2)*(x(i+1,3)+(x(i,3))));
    ceq_q2(i,1) = x(i+1,2)-x(i,2)-((h/2)*(x(i+1,4)+(x(i,4))));
end
ceq_5 = x(1,1)+ pi/2;
ceq_6 = x(1,2);
ceq_7 = x(1,3);
ceq_8 = x(1,4);
ceq_9 = x(end,1)- pi/2;
ceq_10 = x(end,2);
ceq_11 = x(end,3);
ceq_12 = x(end,4);
ceq = [ceq_dq1;ceq_dq2;ceq_q1;ceq_q2;ceq_5;ceq_6;ceq_7;ceq_8;ceq_9;ceq_10;ceq_11;ceq_12];
c = [];
end
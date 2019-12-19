function f = objfcn(x)
%Object function to be minimized
%   Outputs the object function for minimizing control effort, i.e. torque
%   input
h = 0.02;
Tau1 = x(:,5); %Extract all the Tau1 values
Tau2 = x(:,6); %Extract all the Tau2 values
Tau_sum = 0;
Tau_mat = [Tau1 Tau2];

for i=1:size(Tau1,1)-1
%     Tau1_sum = Tau1_sum + (Tau_mat(i,1)^2) + (Tau_mat(i+1,1)^2);
%     Tau2_sum = Tau2_sum + (Tau_mat(i,2)^2) + (Tau_mat(i+1,2)^2);
    Tau_sum = Tau_sum + Tau_mat(i,:)*Tau_mat(i,:)' + Tau_mat(i+1,:)*Tau_mat(i+1,:)';
end
f = (h/2)*Tau_sum;
end


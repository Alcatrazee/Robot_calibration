function A = A_matrix( twist_matrix,theta )
%   A_MATRIX Summary of this function goes here
%   Detailed explanation goes here
%   This is one 6 x 42  block row of the final A matrix

% create a A-sized matrix
A = zeros(6,42);    
% calculate FK for each joint
[~,~,T_link_abs] = FK_new(twist_matrix,theta);
% calculate first to sixth row of A matrix 
A(:,1:6) = theta(1)*int_Adjoint(twist_matrix(:,1)*theta(1));
for i = 1:6
    A(:,1+i*6:6+i*6) = theta(i+1)*Adjoint(T_link_abs(:,:,i)) * int_Adjoint(twist_matrix(:,i+1)*theta(i+1));
end



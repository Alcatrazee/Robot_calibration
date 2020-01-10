function [T,T_link_incr,T_link_abs] = FK( twist_matrix,theta )

% FK_NEW Summary of this function: calculate forward kinematics of the robot
% input parameters:twist_matrix,theta
% output parameters:[T,T_link_incr,T_link_abs]
% input parameters explanation: 
% twist_matrix: matrix consist of all twist vetor
% theta: angle position of each angle
% output parameters explanation:
% T: transformation matrices from end effector to base
% T_link_incr: transformation matrices for Jacobian calculation 
% T_link_abs: transformation matrices of each joint

% link transformation incremental, i.e., T1, T2, T3, ....
T_link_incr = zeros(4,4,6);
% link transformation absolute, i.e., T1, T1*T2, T1*T2*T3, ...
T_link_abs = zeros(4,4,6);
for i = 1:6
    T_link_incr(:,:,i) = T_matrix(twist_matrix(:,i)*theta(i)); % calculate increamental T
    T_link_abs(:,:,i) = eye(4);                                % declearation of absolute T
    for j = 1:i
        T_link_abs(:,:,i) = T_link_abs(:,:,i) * T_link_incr(:,:,j); % calculate absolute T
    end
end
    T = T_link_abs(:,:,6);  % calculate final T(ee2base)
end
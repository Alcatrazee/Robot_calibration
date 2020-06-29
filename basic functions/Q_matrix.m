function Q = Q_matrix(twist_matrix,theta,measuring_type,ball_No)

if measuring_type==1
    Q = zeros(6,42);
elseif measuring_type==2
    Q = zeros(6,54);
end
Adg = zeros(6,6,7);
Adg(:,:,1) = eye(6);

[~,~,T_abs] = FK( twist_matrix,theta );
for i=2:7
   Adg(:,:,i) = Adjoint(T_abs(:,:,i-1));
end

for i=1:6
    Q(:,6*i-5:6*i) = Adg(:,:,i) - Adg(:,:,i+1); 
end

if measuring_type==1
    Q(:,37:end) = Adg(:,:,7);
elseif measuring_type==2
    Q(:,37+(ball_No-1)*6:37+(ball_No)*6-1) = Adg(:,:,7);
end

end

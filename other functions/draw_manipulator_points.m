function draw_manipulator_points( twist_matrix,Pc0,color )
%DRAW_ROBOT Summary of this function goes here
%   Detailed explanation goes here

hold on;
view([45 35]);
axis equal;
q_vec = cross(twist_matrix(4:6,1:6),twist_matrix(1:3,1:6));
[T,~,~] = FK(twist_matrix,[zeros(1,6),1]);
Pc = T*Pc0;

for i = 1:6
    point_set = [q_vec(:,i) - 100*twist_matrix(4:6,i),q_vec(:,i) + 100*twist_matrix(4:6,i)]';
    plot3(point_set(:,1),point_set(:,2),point_set(:,3),color);
end

plot3(Pc(1),Pc(2),Pc(3),color);
hold off;
end


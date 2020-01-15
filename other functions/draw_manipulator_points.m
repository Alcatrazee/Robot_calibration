function draw_manipulator_points( twist_matrix,Pc0,color )
%DRAW_ROBOT Summary of this function goes here
%   Detailed explanation goes here

hold on;
view([45 35]);
axis equal;
q_vec = cross(twist_matrix(4:6,1:6),twist_matrix(1:3,1:6));
[T,~,~] = FK(twist_matrix,[zeros(1,6),1]);
Pc1 = T*Pc0(:,1);
Pc2 = T*Pc0(:,2);
Pc3 = T*Pc0(:,3);

for i = 1:6
    point_set = [q_vec(:,i) - 100*twist_matrix(4:6,i),q_vec(:,i) + 100*twist_matrix(4:6,i)]';
    plot3(point_set(:,1),point_set(:,2),point_set(:,3),color);
end

plot3(Pc1(1),Pc1(2),Pc1(3),'*r');
plot3(Pc2(1),Pc2(2),Pc2(3),'*g');
plot3(Pc3(1),Pc3(2),Pc3(3),'*b');
line([Pc1(1) Pc2(1)],[Pc1(2) Pc2(2)],[Pc1(3) Pc2(3)])
line([Pc1(1) Pc3(1)],[Pc1(2) Pc3(2)],[Pc1(3) Pc3(3)])
% for i = 1:3
%     point_set = [g_st0(1:3,4),g_st0(1:3,4) + g_st0(1:3,i)]';
%     plot3(point_set(:,1),point_set(:,2),point_set(:,3),color);
% end
hold off;
end


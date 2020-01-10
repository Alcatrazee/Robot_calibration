function draw_manipulator_my( twist_matrix,theta7,color )
%DRAW_ROBOT Summary of this function goes here
%   Detailed explanation goes here

hold on;
view([45 35]);
axis equal;
q_vec = cross(twist_matrix(4:6,1:6),twist_matrix(1:3,1:6));
[g_st0,~,~] = FK_new(twist_matrix,[zeros(1,6),theta7]);

for i = 1:6
    point_set = [q_vec(:,i) - 100*twist_matrix(4:6,i),q_vec(:,i) + 100*twist_matrix(4:6,i)]';
    plot3(point_set(:,1),point_set(:,2),point_set(:,3),color);
end
color = ['r','g','b'];
for i = 1:3
    point_set = [g_st0(1:3,4),g_st0(1:3,4) + 100*g_st0(1:3,i)]';
    plot3(point_set(:,1),point_set(:,2),point_set(:,3),color(i));
end
hold off;
end


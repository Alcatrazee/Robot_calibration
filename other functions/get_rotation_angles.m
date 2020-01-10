function [radius,angles] = get_rotation_angles(points)

[center,rotation_axis] = findCircleCenter(points);
vec = zeros(3,3);
for i=1:3
    vec(:,i) = points(:,i) - center;
end
angles = zeros(1,2);
angles(1) = rad2deg(acos(vec(:,1)'*vec(:,2)/(norm(vec(:,1))*norm(vec(:,2)))));
angles(2) = rad2deg(acos(vec(:,2)'*vec(:,3)/(norm(vec(:,2))*norm(vec(:,3)))));
radius = norm(vec(:,1));
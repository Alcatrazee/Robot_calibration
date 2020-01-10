function [T_a,thetas] = retrive_data(poses_file,angles_file)

a = importdata(poses_file);
thetas = importdata(angles_file);

size_of_angles = size(thetas);
num_of_poses = size_of_angles(1);
T_a = zeros(4,4,num_of_poses);
for i=1:num_of_poses
   T_a(:,:,i) = getReferenceFrame([a(3*i-2,1:3);a(3*i-1,1:3);a(3*i,1:3)]',3);
end
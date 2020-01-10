random_angles = rad2deg(randn(13,6));
rows = length(random_angles);
fid = fopen('real_angles.txt','w');
for i=1:rows
    fprintf(fid,'%0.3f ',random_angles(i,:));
    fprintf(fid,'\r');
end
fclose(fid);
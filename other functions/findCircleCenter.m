function [center,rotation_axis] = findCircleCenter(points)
% This function is for center of a circle calulation with 3 points.
% Input a matrix of three column vectors like [p1 p2 p3],and for each point
% the form [x y z]' is needed.

%p1 = [2042.704860 -427.044627 449.340352]';         %points(1:3,1);
%p2 = [2219.209567  -565.188084  449.340352]';       %points(1:3,2);
%p3 = [1958.919  -219.157  449.340]';                %points(1:3,3);

p1 = points(:,1);
p2 = points(:,2);
p3 = points(:,3);
pn = cross(p1-p2,p1-p3);                            % normal vector of plane

A = [2*(p1-p2)';2*(p1-p3)';pn'];                    % A matrix formulation
B = [sum(p1.^2-p2.^2);sum(p1.^2-p3.^2);(p1'*pn)];   % B vector formulation

center  = A\B;
rotation_axis = pn/norm(pn);

function T = getReferenceFrame(positions,method)
% calculate the reference frame with 3 points, first point is the origin of
% the frame, and second frame is on Y axis and the third point is on X
% axis.

%% method 1
% for method 1, point 1 is the origin of the frame, point 2 is on Y axis,
% point is near X axis.
if method==1
%     origin = [2042.704860 -427.044627 449.340352]';
%     Y_point = [2219.209567  -565.188084  449.340352]';
%     X_point = [1958.919  -219.157  449.340]';
    origin = positions(1:3,1);
    Y_point = positions(1:3,2);
    X_point = positions(1:3,3);

    
    X = (origin-X_point)/norm(origin-X_point);
    Y_wave = (Y_point - origin)/norm(Y_point - origin);
    Y = Y_wave - abs(Y_wave'*X)*X;
    Z = cross(X,Y);
    
    T = [X Y Z origin;0 0 0 1];
end
%% method 2
% for method 2, point 1 and point 2 form a line, and point 3 is the point
% out of the line.
%                   (point n)
%    (point1)*-------*----*(point2)      *------->   y
%                    |                   | 
%                    |                   |
%                    *(point3)           * x
if method==2
    p1 = positions(1:3,1);
    p2 = positions(1:3,2);
    p3 = positions(1:3,3);
    
    den = sum((p1 - p2).^2);
    num = (p1-p2)'*(p1-p3);
    k = num/den;
    pn = p1 - k*(p1-p2);
    
    X = (p3 - pn)/norm(p3 - pn);
    Y = (p2 - pn)/norm(p2 - pn);
    Z = cross(X,Y);
    
    T = [X Y Z pn;0 0 0 1];
end
%% method 3
% for method 1, point 1 is the origin of the frame, point 2 is on Y axis,
% point is near X axis.
if method==3
    origin = positions(1:3,1);
    Y_point = positions(1:3,2);
    X_point = positions(1:3,3);

    
    X = (X_point - origin)/norm(X_point - origin);
    Y_wave = (Y_point - origin)/norm(Y_point - origin);
    Y = Y_wave - abs(Y_wave'*X)*X;
    Z = cross(X,Y);
    
    T = [X Y Z origin;0 0 0 1];
end
function twist = get_twist_from_rotations()

points = importdata('axes_rotate.txt');
points = points';

T_tracker_ref = [
        -0.999445950779616       -0.0254246088092193     -0.000630779049031231               3202.812988
        0.0332768508845761         -0.99969936634656       0.00104865312555405                455.452148
        -0.000665331072367819       0.00103230929346972          0.99999153460816               -618.800415
                         0                         0                         0                         1];

q = zeros(3,6);
w = zeros(3,6);
v = zeros(3,6);
for i=1:6
    [q(:,i),w(:,i)] = findCircleCenter(points(:,i*3-2:i*3)); 
    temp = T_tracker_ref\[q(:,i);1];
    q(:,i) = temp(1:3);                                 % 
    w(:,i) = T_tracker_ref(1:3,1:3)\w(:,i);
    w(:,i) = w(:,i)/norm(w(:,i));                       % normalize w
    v(:,i) = -cross(w(:,i),q(:,i));
end
twist = [v;w];
% for i=1:6
%     twist(:,i) = vee(T_tracker_ref\hat(twist(:,i))*T_tracker_ref);
% end

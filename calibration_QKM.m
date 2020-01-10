%% close all unecessary windows
close all
clear;
clc
 
%% referenrce frame establisment
SMR_poses = importdata('ref.txt');
T_tracker_ref = getReferenceFrame([SMR_poses(1,:);SMR_poses(2,:);SMR_poses(3,:)]',1);

%% parameters decleration
% norminal length of links
length_of_links = [491 350 350 84];                              

% norminal q 
q_vec_0 = [ -590 410 300;                         
            -590 410 length_of_links(1);
            -590 410 length_of_links(1)+length_of_links(2);
           length_of_links(3)-590 410 length_of_links(1)+length_of_links(2);
           length_of_links(3)-590 410 length_of_links(1)+length_of_links(2);
           length_of_links(3)-590 410 length_of_links(1)+length_of_links(2)]';

% norminal w
w_vec_0 = [0 0 1;
           0 1 0;
           0 1 0;
           1 0 0;
           0 1 0;
           1 0 0]';
%% g_st0 calculation(or M matrix from Park's paper)
SMR_initial_file_name = 'init_SMRs.txt';
angles_initial_file_name = 'init_ang.txt';
[g_st0,~] = retrive_data(SMR_initial_file_name,angles_initial_file_name);   % read initial EE posture
g_st0_in_tracker = g_st0;
g_st0 = T_tracker_ref\g_st0;                                                % represents in reference frame

% calculate log(g_st0)
theta_M = norm(log_my(g_st0));
normed_M = log_my(g_st0)/theta_M;       
normed_M(1:3) = normed_M(1:3) - normed_M(1:3)'*normed_M(4:6)*normed_M(4:6);

%% norminal twist_matrix_0; size = 6 x 7
% twist_iden = get_twist_from_rotations();
% twist_matrix_0 = [twist_iden normed_M];
twist_matrix_0 = [[cross(q_vec_0,w_vec_0);w_vec_0],normed_M];               % nominal twist
twist_matrix_copy = twist_matrix_0;

%% read SMR positions and joint angles from files
data_file_name = 'POSES.txt';
angle_file_name = 'AnglesInDeg.txt';
[samples,theta_random_vec] = retrive_data(data_file_name,angle_file_name);  % read and process into postures of end effector
theta_random_vec = deg2rad(theta_random_vec);
num_of_pts = length(theta_random_vec);  
theta_random_vec(:,7) = ones(num_of_pts,1)*theta_M;                         % the 7th column shall be set to thetaM
theta_random_vec(:,3) = theta_random_vec(:,3) - ones(num_of_pts,1)*pi/2;

% transfor SMR position into reference frame.
for i=1:num_of_pts
    samples(:,:,i) = T_tracker_ref\samples(:,:,i);
end

%% variables declaration
df_f_inv = zeros(num_of_pts*6,1);                                           % df*f^-1
dp = zeros(num_of_pts*6,1);                                                 % deviation of configuration parameters
A = zeros(num_of_pts*6,42);                                                 % A matrix
j = 0;                                                                      % iteration time
norm_dp=[];                                                                 % for norm of dp visialization
Jacobian = zeros(6,6);
%% parameters identification and composition
while j<20
    %% calculate A matrix and df*f^-1 (parameters identification)
    for i=1:num_of_pts                                                      % repeat num_of_pts times
        [T_n,~,T_n_abs] = FK_new(twist_matrix_0,theta_random_vec(i,:));           % Tn calculation
        T_a = samples(:,:,i);                                               % in this case,we have ee postures represent in reference frame
        A(1+i*6-6:i*6,:) = A_matrix(twist_matrix_0,theta_random_vec(i,:));  % A matrix calculation
        df_f_inv(1+i*6-6:i*6) = log_my(T_a/T_n);                            % solve for log(df_f_inv)
%         for k = 1:6
%             Jacobian(:,k) = vee(T_n_abs(:,:,k)*hat(twist_matrix_0(:,k)));
%         end
%         det(Jacobian)
    end
    
    condition_number = norm(A)*norm(pinv(A))
    dp = A\df_f_inv;                                                        % solve for dp(derive of twist)
    %% composition
    for i=1:6
        twist_matrix_0(:,i) = twist_matrix_0(:,i) + dp(1+i*6-6:i*6,1);                                                          % composition
        twist_matrix_0(:,i) = twist_matrix_0(:,i)/norm(twist_matrix_0(4:6,i));                                                  % normalization
        twist_matrix_0(1:3,i) = twist_matrix_0(1:3,i) - twist_matrix_0(1:3,i)'*twist_matrix_0(4:6,i)*twist_matrix_0(4:6,i);     % make v perpendicular to w
    end
    twist_matrix_0(:,7) = twist_matrix_0(:,7)+dp(37:42,1);                  % alternate the last rows
    %% data prepration of visialization
    j=j+1;                                                                  % counter plus 1 
    norm_dp = [norm_dp norm(dp)];                                           % minimization target value calculation
    disp (norm(dp))                                                         % show value of norm of dp
    disp (j)                                                                % show number of iteration
    if norm(dp) < 0.00009                                                   % quit the for loop if deviation is less than 1e-5
        break;
    end
    %% plot
    clf;                                                                    % clear plot
    draw_manipulator_my(twist_matrix_0,theta_M,'b');                        % draw nominal axis
    drawnow;
end
%% plot again
fig2 = figure(2);                                                           % create another window
bar3(norm_dp)                                                               % plot discrete data

%% verify the new twist
% read SMR poses and joint angles from file
data_file_name = 'POSES.txt';
angle_file_name = 'AnglesInDeg.txt';
[test_poses,test_angles] = retrive_data(data_file_name,angle_file_name);     % get postures and joint angles from file
angle_matrix_size = size(test_angles);
num_of_test_points = angle_matrix_size(1);
test_angles = [deg2rad(test_angles) ones(angle_matrix_size(1),1)*theta_M];                     
test_angles(:,3) = test_angles(:,3)  - ones(num_of_test_points,1)*pi/2;
% variables decleration
truth_poses = zeros(4,4,num_of_test_points);
estimated_poses = zeros(4,4,num_of_test_points);
deviation = zeros(4,4,num_of_test_points);                                  % quotient of measured value and estimated value
delta_eps = zeros(6,num_of_test_points);
norm_delta_eps = delta_eps;
delta_theta = zeros(1,num_of_test_points);
dis_deviation_sum = 0;

% start testing
for i=1:num_of_test_points
    truth_poses(:,:,i) = T_tracker_ref\test_poses(:,:,i);                   % transform reference frame
%     iden_poses(:,:,i) = FK_new(twist_iden,test_angles(i,:));
    [estimated_poses(:,:,i),~,~] = FK_new(twist_matrix_0,test_angles(i,:)); % estimate poses of EE with the same angles
    deviation(:,:,i) = truth_poses(:,:,i)/estimated_poses(:,:,i);           % calculate deviation of two poses
%     deviation_iden(:,:,i) = truth_poses(:,:,i)/iden_poses(:,:,i);
    
    dis_deviation_sum = dis_deviation_sum + norm(deviation(1:3,4,i));
    delta_eps(:,i) = log_my(deviation(:,:,i));   % norm of deviation of gesture
    delta_theta(i) = norm(delta_eps(:,i));
    norm_delta_eps(:,i) = delta_eps(:,i)/norm(delta_eps(:,i));
end
disp 'average delta theta (rad)'
norm(delta_theta)
disp 'average omega (mm)'
norm(norm_delta_eps(4:6,:))
view(-60,20)
q_origin = zeros(3,6);
q_after = zeros(3,6);
for i = 1:6
    q_after(:,i) = cross(twist_matrix_0(4:6,i),twist_matrix_0(1:3,i));
    q_origin(:,i) = cross(twist_matrix_copy(4:6,i),twist_matrix_copy(1:3,i));
end
disp 'average position error'
dis_deviation_sum/num_of_test_points



%% statements
%  This script is to calibrate a QKM robot HL6 with a FARO laser tracker in
%   two different ways, change the measuring type in line 9, which number 1
%   represents pose measurment, and number 2 represents position measurment
%   where both use three SMRs.
%
%  Both measurments used a user frame outside of the robot, therefore there
%   will be a process of frame transforming from laser tracker's coordinate
%   and user's frame, due to using of user frame, the initial guess of twist
%   contains offset of value q.

%% close all unecessary windows
close all
clear;
clc

%% measurment type selector

measurment_type = 1;        % 1: pose measurment   2: position measurment
num_of_SMRs = 3;            % amount of SMRs, at least three to form a coordinate

%% referenrce frame establisment
if measurment_type == 1
    SMR_poses = importdata('ref.txt');
    T_tracker_ref = getReferenceFrame([SMR_poses(1,:);SMR_poses(2,:);SMR_poses(3,:)]',1);
elseif measurment_type == 2
    T_tracker_ref =  getReferenceFrame(importdata('re_ref.txt')',1);
end
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
if measurment_type == 1
    SMR_initial_file_name = 'init_SMRs.txt';
    angles_initial_file_name = 'init_ang.txt';
    [g_st0,~] = retrive_data(SMR_initial_file_name,angles_initial_file_name);   % read initial EE posture
    g_st0_in_tracker = g_st0;
    g_st0 = T_tracker_ref\g_st0;                                                % represents in reference frame
    
    % calculate log(g_st0)
    theta_M = norm(log_my(g_st0));
    normed_M = log_my(g_st0)/theta_M;
    normed_M(1:3) = normed_M(1:3) - normed_M(1:3)'*normed_M(4:6)*normed_M(4:6);
    
elseif measurment_type == 2
    
    PC0 = importdata('re_smr.txt');
    P_c0_n_1 = T_tracker_ref\[PC0(1,:)';1];                                     % frame transfermation
    P_c0_n_2 = T_tracker_ref\[PC0(2,:)';1];
    P_c0_n_3 = T_tracker_ref\[PC0(3,:)';1];
    
end

%% read SMR positions and joint angles from files
if measurment_type == 1
    
    twist_matrix_0 = [[cross(q_vec_0,w_vec_0);w_vec_0],normed_M];               % nominal twist
    twist_matrix_copy = twist_matrix_0;                                         % copy a twist for comparison
    [samples,theta_random_vec] = retrive_data('POSES.txt','AnglesInDeg.txt');   % read and process into postures of end effector
    theta_random_vec = deg2rad(theta_random_vec);
    num_of_pts = length(theta_random_vec);
    theta_random_vec(:,7) = ones(num_of_pts,1)*theta_M;                         % the 7th column shall be set to thetaM
    theta_random_vec(:,3) = theta_random_vec(:,3) - ones(num_of_pts,1)*pi/2;    % due to modeling difference, initial value of mathermatical model has a offset
    
elseif measurment_type == 2
    
    twist_matrix_0 = [cross(q_vec_0,w_vec_0);w_vec_0];                          % nominal twist
    twist_matrix_copy = twist_matrix_0;
    theta_random_vec_deg = importdata('c_angles.txt');                          % import angle data from file, the input file is of degree, so we need to transform them into radias
    num_of_pts = length(theta_random_vec_deg);                                  % get number of points
    theta_random_vec = deg2rad(theta_random_vec_deg(:,1:6));                    % unit transformation
    samples = [importdata('c_poses.txt') ones(num_of_pts,1)]';                  % import measured data
    theta_random_vec(:,3) = theta_random_vec(:,3) - ones(num_of_pts,1)*pi/2;    % offset correction
end

% transfor SMR position into reference frame.
for i=1:num_of_pts
    if measurment_type == 1
        samples(:,:,i) = T_tracker_ref\samples(:,:,i);
    elseif measurment_type == 2
        samples(:,i) = T_tracker_ref\samples(:,i);
    end
end

%% variables declaration

if measurment_type == 1
    df_f_inv = zeros(num_of_pts*6,1);
    A = zeros(num_of_pts*6,42);
elseif measurment_type == 2
    df_f_inv = zeros(num_of_pts*3,1);
    A = zeros(num_of_pts*3,36+num_of_SMRs*6);                               % A matrix
    norm_dpc = [];                                                          % norm of deviation of position
    dPc_matrix = zeros(3,num_of_pts);                                       % for norm of dpc calculation
end

j = 0;                                                                      % iteration time
norm_k=[];                                                                  % for norm of dk visialization
SMR_index = 0;

%% parameters identification and composition

while j<20
    %% calculate A matrix and df*f^-1 (parameters identification)
    for i=1:num_of_pts                                                      % repeat num_of_pts times
        [T_n,~,~] = FK(twist_matrix_0,theta_random_vec(i,:));               % T_n calculation
        
        if measurment_type == 1
            Q = Q_matrix(twist_matrix_0,theta_random_vec(i,:),measurment_type,SMR_index);             % Q matrix calculation
            T_n = T_n*g_st0;
            T_a = samples(:,:,i);
            df_f_inv(1+i*6-6:i*6) = log_my(T_a/T_n);
            A(1+i*6-6:i*6,:) = Q;
        elseif measurment_type == 2
            SMR_index = theta_random_vec_deg(i,7);                          % get index of SMR
            Q = Q_matrix(twist_matrix_0,theta_random_vec(i,:),measurment_type,SMR_index);             % Q matrix calculation
            if(SMR_index==1)
                P_n = T_n * P_c0_n_1;
            elseif(SMR_index==2)
                P_n = T_n * P_c0_n_2;
            else
                P_n = T_n * P_c0_n_3;
            end
            
            P_a = samples(:,i);
            dpc = P_a - P_n;                                                % deviation of actual position and norminal position                                         %
            dPc_matrix(:,i) = dpc(1:3);
            
            df_f_inv(i*3-2:i*3) = dpc(1:3);
            A(3*i-2:3*i,:) = [eye(3) -hat(P_n(1:3))]*Q;
        end
        
    end
    B = B_matrix(twist_matrix_0,measurment_type,num_of_SMRs);               % amazing matrix
    k = (A*B)\df_f_inv;
    norm(k)
    
    % composition of twist
    for i=1:6
        twist_matrix_0(:,i) = Adjoint(T_matrix(B(i*6-5:i*6,i*4-3:i*4)*k(i*4-3:i*4)))*twist_matrix_0(:,i);
    end
    if(measurment_type==1)
        g_st0 = T_matrix(k(25:30))*g_st0;
        twist_matrix_0(:,7) = log_my(g_st0);
        gst_n = g_st0;
    elseif(measurment_type==2)
        P_c0_n_1 = [k(25:27)+P_c0_n_1(1:3);1];                                  % composite smr positions in initial configuration
        P_c0_n_2 = [k(28:30)+P_c0_n_2(1:3);1];
        P_c0_n_3 = [k(31:33)+P_c0_n_3(1:3);1];
    end
    
    j=j+1;                                                                      % counter plus 1
    norm_k = [norm_k norm(k)];                                                  % minimization target value calculation
    disp (norm(k))                                                              % show value of norm of dp                                                         % show number of iteration
    
    if measurment_type == 2
        norm_dpc = [norm_dpc norm(dPc_matrix)];                                 % calculate norm of deviation of positions
        disp 'norm of dPc'
        disp (norm_dpc(j))
    end
    
    %% plot
    clf;
    if measurment_type == 1
        draw_manipulator(twist_matrix_0,gst_n,'b',measurment_type);                 % draw robot frame with norminal twist
    elseif measurment_type == 2
        draw_manipulator_points(twist_matrix_0,[P_c0_n_1,P_c0_n_2,P_c0_n_3],'b');   % draw robot frame with norminal twist
    end
    drawnow;
    if norm(k) < 10e-11                                                             % judge if it's able to quit the iteration
        break;
    elseif (norm(k)>1e4)||j>100
        break;
    end
end
%% plot again

if measurment_type == 1
    fig2 = figure(2);                                                           % create another window
    bar(norm_k)                                                                 % plot discrete data
%     view(-60,20)                                                                % adjust cam
    legend('||dk||')
    xlabel('num of iteration')
    ylabel('||dk||')
    for i=1:length(norm_k)
       text(i-0.35,norm_k(i)+max(norm_k)*0.05,num2str(norm_k(i),'%3.3f'))
    end
    yaxis([0 max(norm_k)*1.1])
    title('dk')
elseif measurment_type == 2
    fig = figure('name','result','position',[200 200 1200 600]);                                                           % create another window
    subplot(1,2,1);
    bar(norm_k)                                                                 % plot discrete data
%     view(-60,20)                                                                % adjust cam
    legend('||dk||')
    xlabel('num of iteration')
    ylabel('||dk||')
    for i=1:length(norm_k)
       text(i-0.35,norm_k(i)+max(norm_k)/40,num2str(norm_k(i),'%3.3f'))
    end
    yaxis([0 max(norm_k)*1.1])
    title('dk')
    
    subplot(1,2,2);
    bar(norm_dpc);
%     view(-60,20)
    legend('||dPc||')                                                             % plot discrete data
    xlabel('num of iteration')
    ylabel('||dPc||')
    for i=1:length(norm_dpc)
       text(i-0.35,norm_dpc(i)+max(norm_dpc)/40,num2str(norm_dpc(i),'%3.3f'))
    end
    yaxis([0 max(norm_dpc)*1.1])
    title('dPc')
end
%% verify the new twist

if measurment_type == 1
    % read SMR poses and joint angles from file
    [test_poses,test_angles] = retrive_data('POSES.txt','AnglesInDeg.txt');     % get postures and joint angles from file
    num_of_test_points = size(test_angles,1);
    test_angles = [deg2rad(test_angles) ones(num_of_test_points,1)*theta_M];
    test_angles(:,3) = test_angles(:,3)  - ones(num_of_test_points,1)*pi/2;     % get rid of offset
    
    % variables decleration
    truth_poses = zeros(4,4,num_of_test_points);
    estimated_poses = zeros(4,4,num_of_test_points);
    deviation = zeros(4,4,num_of_test_points);                                  % quotient of measured value and estimated value
    delta_eps = zeros(6,num_of_test_points);
    norm_delta_eps = delta_eps;
    delta_theta = zeros(1,num_of_test_points);
    pose_deviation = zeros(3,num_of_test_points);
    % start testing
    for i=1:num_of_test_points
        truth_poses(:,:,i) = T_tracker_ref\test_poses(:,:,i);                   % transform reference frame
        [estimated_T,~,~] = FK(twist_matrix_0,test_angles(i,:));                % estimate poses of EE with the same angles
        estimated_poses(:,:,i) = estimated_T* gst_n;
        deviation(:,:,i) = truth_poses(:,:,i)/estimated_poses(:,:,i);           % calculate deviation of two poses
        
        pose_deviation(:,i) = deviation(1:3,4,i);
        delta_eps(:,i) = log_my(deviation(:,:,i));                              % norm of deviation of gesture
        delta_theta(i) = norm(delta_eps(:,i));
        norm_delta_eps(:,i) = delta_eps(:,i)/delta_theta(i);
    end
    disp 'average dtheta (rad)'
    disp(norm(delta_theta))
    disp 'average domega (mm)'
    disp(norm(norm_delta_eps(4:6,:)))
    
    %     q_origin = zeros(3,6);
    %     q_after = zeros(3,6);
    %     for i = 1:6
    %         q_after(:,i) = cross(twist_matrix_0(4:6,i),twist_matrix_0(1:3,i));
    %         q_origin(:,i) = cross(twist_matrix_copy(4:6,i),twist_matrix_copy(1:3,i));
    %     end
    disp 'average position error'
    disp(mean(vecnorm(pose_deviation)))
    disp 'max  position error'
    disp(max(vecnorm(pose_deviation)))
    
elseif measurment_type == 2
    
    ball_index = 7;
    test_angles_deg = importdata('test_points.txt');
    num_of_test_points = length(test_angles_deg);
    test_angles = deg2rad(test_angles_deg);                                     % you know what it is
    truth_positions = [importdata('re_test.txt') ones(num_of_test_points,1)]';
    test_angles(:,3) = test_angles(:,3) - ones(num_of_test_points,1)*pi/2;      % for FK calculation
    
    for i=1:num_of_test_points
        truth_positions(:,i) = T_tracker_ref\truth_positions(:,i);
    end
    
    % variables decleration
    estimated = zeros(4,num_of_test_points);
    deviation = zeros(4,num_of_test_points);                                  % quotient of measured value and estimated value
    delta_theta = zeros(1,num_of_test_points);
    
    % start testing
    for i=1:num_of_test_points
        [estimated,~,~] = FK(twist_matrix_0,test_angles(i,:)); % estimate poses of EE with the same angles
        SMR_index = test_angles_deg(i,ball_index);
        %     SMR_index = mod(i,3)+1;
        if(SMR_index==1)
            Pc_n = estimated * P_c0_n_1;
        elseif(SMR_index==2)
            Pc_n = estimated * P_c0_n_2;
        else
            Pc_n = estimated * P_c0_n_3;
        end
        Pc_a = truth_positions(:,i);
        deviation(:,i) = Pc_a - Pc_n;           % calculate deviation of two poses
    end
    disp 'norm of distance deviation:'
    norm(deviation(1:3,:))
    disp 'mean of error'
    mean(vecnorm(deviation(1:3,:)))
    disp 'max of error'
    max(vecnorm(deviation(1:3,:)))
    
end
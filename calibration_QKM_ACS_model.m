%% brief of this program
% This script calibrate twists of a QKM robot using measured data.

%% close all unecessary windows
close all
clear;
clc

%% referenrce frame establisment
T_tracker_ref =  getReferenceFrame(importdata('re_ref.txt')',1);

%% parameters decleration
% norminal length of links
length_of_links = [491 350 350 84];

% norminal q
q_vec_0 = [ -600 400 300;
    -600 400 length_of_links(1);
    -600 400 length_of_links(1)+length_of_links(2);
    length_of_links(3)-600 400 length_of_links(1)+length_of_links(2);
    length_of_links(3)-600 400 length_of_links(1)+length_of_links(2);
    length_of_links(3)-600 400 length_of_links(1)+length_of_links(2)]';

% norminal w
w_vec_0 = [0 0 1;
    0 1 0;
    0 1 0;
    1 0 0;
    0 1 0;
    1 0 0]';

% norminal twist_matrix_0; size = 6 x 6
twist_matrix_n = [cross(q_vec_0,w_vec_0);w_vec_0];               % nominal twist definition
twist_matrix_copy = twist_matrix_n;

%% initial  calculation
HomePosition_offset = [0 0 90 0 0 0]';
PC0 = importdata('re_smr.txt');
P_c0_n_1 = T_tracker_ref\[PC0(1,:)';1];                                     % frame transfermation
P_c0_n_2 = T_tracker_ref\[PC0(2,:)';1];
P_c0_n_3 = T_tracker_ref\[PC0(3,:)';1];
% P_c0_n_1 = [-130,411,940 1]';
% P_c0_n_2 = [-130,513,841 1]';
% P_c0_n_3 = [-130,293,841 1]';

%% read SMR positions and joint angles from files
theta_random_vec_deg = importdata('c_angles.txt');                          % import angle data from file
num_of_pts = length(theta_random_vec_deg);                                  % get number of points
theta_random_vec = deg2rad(theta_random_vec_deg(:,1:6));                    % translate unit
samples = [importdata('c_poses.txt') ones(num_of_pts,1)]';                  % import measured data
theta_random_vec(:,3) = theta_random_vec(:,3) - ones(num_of_pts,1)*pi/2;    % offset correction

% transform SMR postures to reference frame.
for i=1:num_of_pts
    samples(:,i) = T_tracker_ref\samples(:,i);                              % coordinate transformation
end

%% variables declaration
j = 0;                                              % iteration counter
dPc = zeros(3*num_of_pts,1);
norm_dpc = [];
dPc_matrix = zeros(3,num_of_pts);
B = zeros(42,4);
Q = zeros(6,42);
norm_k=[];                              % for norm of dp visialization
A = zeros(num_of_pts*3,54);

%% parameters identification and composition
while j<1000
    %% calculate A_tilde matrix and dPc (parameters identification)
    for i=1:num_of_pts                                                      % repeat num_of_pts times
        [Tn,~,~] = FK(twist_matrix_n,theta_random_vec(i,:));                % Tn calculation
        num_of_ball = theta_random_vec_deg(i,7);                            % index of SMR to be detected
        if(num_of_ball==1)
            P_n = Tn * P_c0_n_1;
        elseif(num_of_ball==2)
            P_n = Tn * P_c0_n_2;
        else
            P_n = Tn * P_c0_n_3;
        end
        
        Q = Q_matrix(twist_matrix_n,theta_random_vec(i,:),num_of_ball);
        P_a = samples(:,i);                                                 % actual SMR position
        dpc = P_a - P_n                                                     % deviation of actual position and norminal position
        norm(dpc)                                                           % calculate norm of dpc
        dPc(i*3-2:i*3) = dpc(1:3);                                          %
        dPc_matrix(:,i) = dpc(1:3);
        A(3*i-2:3*i,:) = [eye(3) -hat(P_n(1:3))]*Q;
    end                                                 % solve for dp(derive of twist)
    B = B_matrix(twist_matrix_n,2);                       % amazing matrix
    k = (A*B)\dPc;
    norm(k)
    %% calculate observibility index
%     [cols,rows] = size(A_tilde);
%     [U,S,V] = svd(A_tilde);
%     V = V';
%     singular_value_mul = 1;
%     for i=1:rank(A_tilde)
%         singular_value_mul = singular_value_mul*S(i,i);
%     end
%     observibility_index = singular_value_mul^(1/rows)/sqrt(num_of_pts)
    %% composition
    for i=1:6
        twist_matrix_n(:,i) = Adjoint(T_matrix(B(i*6-5:i*6,i*4-3:i*4)*k(i*4-3:i*4)))*twist_matrix_n(:,i);
    end
    P_c0_n_1 = [k(25:27)+P_c0_n_1(1:3);1];                          % composite smr positions in initial configuration
    P_c0_n_2 = [k(28:30)+P_c0_n_2(1:3);1];
    P_c0_n_3 = [k(31:33)+P_c0_n_3(1:3);1];
    %% data prepration of visialization
    j=j+1;                                                                  % counter plus 1
    norm_k = [norm_k norm(k)];                                   % calculate d_eta and push back to a array
    norm_dpc = [norm_dpc norm(dPc_matrix)];                                 % calculate norm of deviation of positions
    disp 'norm of dPc'
    disp (norm_dpc(j))                                                      % display deviation of norminal and actual position of a SMR
    disp 'norm of d_eta'
    disp (norm(k))                                                      % show value of norm of d_eta
    disp (j)                                                                % show number of iteration
    % plot
    clf;                                                                    % clear plot
    draw_manipulator_points(twist_matrix_n,[P_c0_n_1,P_c0_n_2,P_c0_n_3],'b');                     % draw nominal axis
    drawnow;
    if norm(k) < 1e-11                                                  % quit the for loop if deviation is less than 1e-5
        break;
    end
end
%% plot again
fig2 = figure(2);                                                           % create another window
bar3(norm_k)                                                            % plot discrete data
view(-60,20)                                                                % adjust cam
legend('||dk||')

fig3 = figure(3);
bar3(norm_dpc);
view(-60,20)
legend('||dPc||')                                                           % plot discrete data
%% verify the new twist
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
    [estimated,~,~] = FK(twist_matrix_n,test_angles(i,:)); % estimate poses of EE with the same angles
    num_of_ball = test_angles_deg(i,ball_index);
    %     num_of_ball = mod(i,3)+1;
    if(num_of_ball==1)
        Pc_n = estimated * P_c0_n_1;
    elseif(num_of_ball==2)
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

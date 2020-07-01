%% brief of this program
% This program is for calibration simulation of robot arm QKM HL6 with
% RoboDK. This program can calibrate the robot automatically, with randomly
% generated joint angles.

% How to use it: open  RoboDK file(cali_sim.rdk),
% then run this script.

%% close all unecessary windows
close all
clear;
clc

%% switches

paper_to_use = 0;           % 0:paper of Prof. Lou's   1:paper of Prof. Wu's
measurment_type = 2;        % 1: pose measurment   2: position measurment
num_of_SMRs = 3;            % amount of SMRs, at least three to form a coordinate

%% RoboDK relative variables
global RDK;
global robot;
global tracker;
global robot_base;
RDK = Robolink();
robot = RDK.Item('QKM HL6');
robot_base = RDK.Item('QKM HL6 Base');
tracker = RDK.Item('Tracker Frame');

%% referenrce frame establisment
T_tracker_ref = Get_ref(RDK,tracker,robot_base);

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
          
%% g_st0 calculation(or M matrix from Park's paper)
HomePosition = [0 0 90 0 0 0]';
robot.MoveJ(HomePosition);
Pc0_1_in_tracker = get_SMR_pos(1);
Pc0_2_in_tracker = get_SMR_pos(2);
Pc0_3_in_tracker = get_SMR_pos(3);
P_c0_n_1 = T_tracker_ref\[Pc0_1_in_tracker;1];                                     % represents in reference frame
P_c0_n_2 = T_tracker_ref\[Pc0_2_in_tracker;1];
P_c0_n_3 = T_tracker_ref\[Pc0_3_in_tracker;1];                                  % represents in reference frame                        

%% norminal twist_matrix_0; size = 6 x 7
twist_matrix_n = [cross(q_vec_0,w_vec_0);w_vec_0];               % nominal twist
twist_matrix_copy = twist_matrix_n;

%% read SMR positions and joint angles from files
num_of_pts = 20;
theta_random_vec = GetRandomAngles(num_of_pts);
% theta_random_vec = importdata('test_angles.txt');
theta_random_vec_deg = rad2deg(theta_random_vec);                  
samples = [MeasurePosition_threeSMR(theta_random_vec_deg);ones(1,num_of_pts)];
theta_random_vec(:,3) = theta_random_vec(:,3) - ones(num_of_pts,1)*pi/2;

% transform SMR postures to reference frame.
for i=1:num_of_pts
    samples(:,i) = T_tracker_ref\samples(:,i);
end


%% variables declaration

j = 0;                                              % iteration time
if paper_to_use==0
    eta_matrix = zeros(6,6);
    d_eta = zeros(45,1);
    dPc = zeros(3*num_of_pts,1);
    A_tilde = zeros(3*num_of_pts,45);
    norm_d_eta = [];
elseif paper_to_use==1
    df_f_inv = zeros(num_of_pts*3,1);
    A = zeros(num_of_pts*3,36+num_of_SMRs*6);                               % A matrix
    norm_k=[];                                                                  % for norm of dk visialization
    SMR_index = 0;
end

norm_dpc = [];
dPc_matrix = zeros(3,num_of_pts);


%% add noise to samples
% samples_noised = samples;
% samples_noised(1:3,:) = samples(1:3,:) + rand(3,num_of_pts)/10;

%% parameters identification and composition
while j<1000
    %% calculate A matrix and df*f^-1 (parameters identification)
    for i=1:num_of_pts                                                      % repeat num_of_pts times
        [Tn,~,~] = FK(twist_matrix_n,theta_random_vec(i,:));                % Tn calculation                                                         
        SMR_index = mod(i,3)+1;
        if(SMR_index==1)
            P_n = Tn * P_c0_n_1;
        elseif(SMR_index==2)
            P_n = Tn * P_c0_n_2;
        else
            P_n = Tn * P_c0_n_3;
        end
        P_a = samples(:,i);
        dpc = P_a - P_n;
        dPc(i*3-2:i*3) = dpc(1:3);
        dPc_matrix(:,i) = dpc(1:3);
        if paper_to_use==0
            A_tilde(i*3-2:i*3,:) = A_tilde_matrix(twist_matrix_n,eta_matrix,P_n,theta_random_vec(i,:),SMR_index-1);  % A matrix calculation
        elseif paper_to_use==1
            Q = Q_matrix(twist_matrix_n,theta_random_vec(i,:),measurment_type,SMR_index);
            df_f_inv(i*3-2:i*3) = dpc(1:3);
            A(3*i-2:3*i,:) = [eye(3) -hat(P_n(1:3))]*Q;
        end
    end
    % difference
    if paper_to_use==0
        d_eta = A_tilde\dPc;                                                          % solve for dp(derive of twist)
    elseif paper_to_use==1
        B = B_matrix(twist_matrix_n,measurment_type,num_of_SMRs);               % amazing matrix
        k = (A*B)\df_f_inv;  
    end
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
        if paper_to_use==0
            eta_matrix(:,i) = eta_matrix(:,i)+d_eta(i*6-5:i*6);
            twist_matrix_n(:,i) = Adjoint(T_matrix(eta_matrix(:,i)))*twist_matrix_copy(:,i);
        elseif paper_to_use==1
            twist_matrix_n(:,i) = Adjoint(T_matrix(B(i*6-5:i*6,i*4-3:i*4)*k(i*4-3:i*4)))*twist_matrix_n(:,i); 
        end
    end
    if paper_to_use==0
        P_c0_n_1 = [P_c0_n_1(1:3) + d_eta(37:39);1];
        P_c0_n_2 = [P_c0_n_2(1:3) + d_eta(40:42);1];
        P_c0_n_3 = [P_c0_n_3(1:3) + d_eta(43:45);1];
    elseif paper_to_use==1
        P_c0_n_1 = [k(25:27)+P_c0_n_1(1:3);1];                                  % composite smr positions in initial configuration
        P_c0_n_2 = [k(28:30)+P_c0_n_2(1:3);1];
        P_c0_n_3 = [k(31:33)+P_c0_n_3(1:3);1];
    end
    %% data prepration of visialization
                                                                      % counter plus 1
    if paper_to_use==0
        norm_d_eta = [norm_d_eta norm(d_eta)];
        disp 'norm of d_eta'
        disp (norm(d_eta))   % show value of norm of dp
    elseif paper_to_use==1
        norm_k = [norm_k norm(k)];                                                  % minimization target value calculation
        disp (norm(k))                                                              % show value of norm of dp                                                         % show number of iteration                              % calculate norm of deviation of positions
    end
    norm_dpc = [norm_dpc norm(dPc_matrix)];
    
    j=j+1;                                               
    disp (j)                                                                % show number of iteration
    
    % plot
    clf;                                                                    % clear plot
    draw_manipulator_points(twist_matrix_n,[P_c0_n_1,P_c0_n_2,P_c0_n_3],'b');                     % draw nominal axis
    drawnow;
    if paper_to_use==0
        if norm(d_eta) < 1e-11                                                  % quit the for loop if deviation is less than 1e-5
            break;
        end
    elseif paper_to_use==1
        if norm(k) < 1e-11                                                             % judge if it's able to quit the iteration
            break;
        elseif (norm(k)>1e4)||j>100
            break;
        end
    end
    
end
%% plot again
if paper_to_use==0
    
    fig2 = figure(2);                                                           % create another window
    bar3(norm_d_eta)                                                            % plot discrete data
    view(-60,20)                                                                % adjust cam 
    legend('||deta||')

    fig3 = figure(3);
    bar3(norm_dpc);
    view(-60,20) 
    legend('||dPc||')                                                             % plot discrete data
elseif paper_to_use==1

    fig = figure('name','result','position',[200 200 1200 600]);                                                           % create another window
    subplot(1,2,1);
    bar(norm_k)                                                                 % plot discrete data                                                               % adjust cam
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
num_of_test_points = 50;
test_angles = GetRandomAngles(num_of_test_points);                          % generate new points
%test_angles = importdata('test_angles.txt');
test_angles_deg = rad2deg(test_angles);                                     % you know what it is
truth_positions = [MeasurePosition_threeSMR(test_angles_deg);ones(1,num_of_test_points)];           % launch sim on RoboDK for data
test_angles(:,3) = test_angles(:,3) - ones(num_of_test_points,1)*pi/2;      % for FK calculation

for i=1:num_of_test_points
    truth_positions(:,i) = T_tracker_ref\truth_positions(:,i);
end

% variables decleration
estimated = zeros(4,num_of_test_points);
deviation = zeros(4,num_of_test_points);                                  % quotient of measured value and estimated value
delta_theta = zeros(1,num_of_test_points);
dis_deviation_sum = 0;

% start testing
for i=1:num_of_test_points
    [estimated,~,~] = FK(twist_matrix_n,test_angles(i,:)); % estimate poses of EE with the same angles
    SMR_index = mod(i,3)+1;
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
disp(norm(deviation(1:3,:)))
disp("calibrated SMRs' position")
disp([P_c0_n_1 P_c0_n_2 P_c0_n_3])

%% some useful functions
% get reference frame using 3 SMRs
function posture = Get_ref(RDK,tracker,robot_base)
    Ref1 = RDK.Item('uRef1');
    Ref1.setParentStatic(tracker)
    posture1 = Ref1.Pose();
    Ref1.setParentStatic(robot_base);
    
    Ref2 = RDK.Item('uRef2');
    Ref2.setParentStatic(tracker)
    posture2 = Ref2.Pose();
    Ref2.setParentStatic(robot_base);
    
    Ref3 = RDK.Item('uRef3');
    Ref3.setParentStatic(tracker)
    posture3 = Ref3.Pose();
    Ref3.setParentStatic(robot_base);
    
    poses = [posture1(1:3,4) posture2(1:3,4) posture3(1:3,4)];
    posture = getReferenceFrame(poses,1);
end

% get end effector's posture
function posture = Get_Calibration_plate_posture()
    SMR1_position =  get_SMR_pos(1);
    SMR2_position =  get_SMR_pos(2);
    SMR3_position =  get_SMR_pos(3);
    SMRs_position = [SMR1_position SMR2_position SMR3_position];
    posture = getReferenceFrame(SMRs_position,3);
end

% calculate SMR poses with respect to reference frame
function pos = get_SMR_pos(SMR_n)
    global RDK;
    global robot;
    global tracker;
    if(SMR_n == 1)
        SMR = RDK.Item('SMR1');
        Pose = [     1.000000,    -0.000000,    -0.000000,  -110.000000 ;
     -0.000000,     1.000000,     0.000000,     0.000000 ;
     -0.000000,     0.000000,     1.000000,    35.000000 ;
      0.000000,     0.000000,     0.000000,     1.000000 ];
    elseif(SMR_n == 2)
        SMR = RDK.Item('SMR2');
        Pose = [      1.000000,    -0.000000,    -0.000000,     0.000000 ;
     -0.000000,     1.000000,     0.000000,   110.000000 ;
     -0.000000,     0.000000,     1.000000,    35.000000 ;
      0.000000,     0.000000,     0.000000,     1.000000 ];
    elseif(SMR_n == 3)
        SMR = RDK.Item('SMR3');
        Pose = [      1.000000,    -0.000000,    -0.000000,     0.000000 ;
     -0.000000,     1.000000,     0.000000,  -110.000000 ;
     -0.000000,     0.000000,     1.000000,    35.000000 ;
      0.000000,     0.000000,     0.000000,     1.000000 ];
    end
    SMR.setParentStatic(tracker)
    posture = SMR.Pose();
    SMR.setParentStatic(robot);
    SMR.setPose(Pose);
    pos = posture(1:3,4);
end

% calculate random angles within joint limit
function theta_random_vec = GetRandomAngles(NumOfPts)
    theta_random_vec = randn(NumOfPts,6);
    Joint_limit = deg2rad([170 110 136 185 120 360]);
    for i = 1:NumOfPts
        for j=1:6
            while(abs(theta_random_vec(i,j))>Joint_limit(j))
                    theta_random_vec(i,j) = randn();
            end
        end
    end
end

% drive the robot in simulator to certain posture and return posture of
% plate's frame
function Pc = MeasurePosition(angles,SMR_n)
    global robot;
    size_of_angles = size(angles);
    NumOfPts = size_of_angles(1);
    Pc = zeros(3,NumOfPts);
    for i = 1:NumOfPts
        robot.MoveJ(angles(i,:));
        Pc(:,i) = get_SMR_pos(SMR_n);
    end
end

function Pc = MeasurePosition_threeSMR(angles)
    global robot;
    size_of_angles = size(angles);
    NumOfPts = size_of_angles(1);
    Pc = zeros(3,NumOfPts);
    for i = 1:NumOfPts
        robot.MoveJ(angles(i,:));
        SMR_n = mod(i,3)+1;
        Pc(:,i) = get_SMR_pos(SMR_n);
    end
end
global RDK;
global robot;
global tracker;
global robot_base;
RDK = Robolink();
robot = RDK.Item('QKM HL6');
robot_base = RDK.Item('QKM HL6 Base');
tracker = RDK.Item('Tracker Frame');

refFrame = Get_ref(RDK,tracker,robot_base);
gst_0_in_tracker = Get_Calibration_plate_posture();

theta_random_vec_rad = randn(10,7);
theta_random_vec_deg = rad2deg(theta_random_vec_rad);



function posture = Get_ref(RDK,tracker,robot_base)
    Ref1 = RDK.Item('Ref1');
    Ref1.setParentStatic(tracker)
    posture1 = Ref1.Pose();
    Ref1.setParentStatic(robot_base);
    
    Ref2 = RDK.Item('Ref2');
    Ref2.setParentStatic(tracker)
    posture2 = Ref2.Pose();
    Ref2.setParentStatic(robot_base);
    
    Ref3 = RDK.Item('Ref3');
    Ref3.setParentStatic(tracker)
    posture3 = Ref3.Pose();
    Ref3.setParentStatic(robot_base);
    
    poses = [posture1(1:3,4) posture2(1:3,4) posture3(1:3,4)];
    posture = getReferenceFrame(poses,2);
end

function posture = Get_Calibration_plate_posture()
    SMR1_position =  get_SMR_pos(1);
    SMR2_position =  get_SMR_pos(2);
    SMR3_position =  get_SMR_pos(3);
    SMRs_position = [SMR1_position SMR2_position SMR3_position];
    posture = getReferenceFrame(SMRs_position,3);
end

function pos = get_SMR_pos(SMR_n)
    global RDK;
    global robot;
    global tracker;
    if(SMR_n == 1)
        SMR = RDK.Item('SMR1');
        Pose = [     1.000000,    -0.000000,    -0.000000,   -70.000000 ;
                     -0.000000,     1.000000,     0.000000,   -70.000000 ;
                     -0.000000,     0.000000,     1.000000,    42.500000 ;
                      0.000000,     0.000000,     0.000000,     1.000000 ];
    elseif(SMR_n == 2)
        SMR = RDK.Item('SMR2');
        Pose = [     1.000000,    -0.000000,    -0.000000,   -70.000000 ;
                     -0.000000,     1.000000,     0.000000,    70.000000 ;
                     -0.000000,     0.000000,     1.000000,    42.500000 ;
                      0.000000,     0.000000,     0.000000,     1.000000 ];
    elseif(SMR_n == 3)
        SMR = RDK.Item('SMR3');
        Pose = [     1.000000,    -0.000000,    -0.000000,    70.000000 ;
                     -0.000000,     1.000000,     0.000000,   -70.000000 ;
                     -0.000000,     0.000000,     1.000000,    42.500000 ;
                      0.000000,     0.000000,     0.000000,     1.000000 ];
    end
    SMR.setParentStatic(tracker)
    posture = SMR.Pose();
    SMR.setParentStatic(robot);
    SMR.setPose(Pose);
    pos = posture(1:3,4);
end
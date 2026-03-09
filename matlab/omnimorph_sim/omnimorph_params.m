%% UAV parameters
config.uavParams.mass = 1.625; % with peripherals and battery
config.uavParams.c_f = 1.5e-4; 
config.uavParams.c_t = 2.2e-6;
config.uavParams.inertia = diag([1.13e-2;1.13e-2;1.13e-2]);
config.uavParams.propTilt = deg2rad(50);

% Nominal/modelled wrench map
config.uavParams.wrenchMap = double(myWrenchMap(config.uavParams.c_t, ...
                                                config.uavParams.c_f , ...
                                                config.uavParams.propTilt, ...
                                                0.2165/sqrt(3)));

% Optimized piece-wise linear wrench maps whith hovering measurement
load('./wMap/Flight/wrenchMap_reg1.mat');
config.uavParams.wMaps(:,:,1) = wMapStruct.wrenchMap(:,:,4);
config.uavParams.wMaps(:,:,2) = config.uavParams.wrenchMap;
load('./wMap/Flight/wrenchMap_reg3.mat');
config.uavParams.wMaps(:,:,3) = wMapStruct.wrenchMap(:,:,4);
load('./wMap/Flight/wrenchMap_reg4.mat');
config.uavParams.wMaps(:,:,4) = wMapStruct.wrenchMap(:,:,4);
config.uavParams.wMaps(:,:,5) = config.uavParams.wrenchMap;
config.uavParams.wMaps(:,:,6) = config.uavParams.wrenchMap;
config.uavParams.wMaps(:,:,7) = config.uavParams.wrenchMap;
config.uavParams.wMaps(:,:,8) = config.uavParams.wrenchMap;
config.uavParams.wMaps(:,:,9) = config.uavParams.wrenchMap;
config.uavParams.wMaps(:,:,10) = config.uavParams.wrenchMap;


%{
% Optimized piece-wise linear wrench maps with FT measurement
load('./wMap/FT_sensor/wrenchMap_reg1.mat');
config.uavParams.wMaps(:,:,1) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg2.mat');
config.uavParams.wMaps(:,:,2) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg3.mat');
config.uavParams.wMaps(:,:,3) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg4.mat');
config.uavParams.wMaps(:,:,4) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg5.mat');
config.uavParams.wMaps(:,:,5) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg6.mat');
config.uavParams.wMaps(:,:,6) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg7.mat');
config.uavParams.wMaps(:,:,7) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg8.mat');
config.uavParams.wMaps(:,:,8) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg9.mat');
config.uavParams.wMaps(:,:,9) = wMapStruct.wrenchMap(:,:,5);
load('./wMap/FT_sensor/wrenchMap_reg10.mat');
config.uavParams.wMaps(:,:,10) = wMapStruct.wrenchMap(:,:,5);
%}

% End effector parameters
config.uavParams.endEffector.p_e = [0;0.48;0]; % EE pos
%config.uavParams.endEffector.p_e = [0;0;0]; % EE pos
config.uavParams.endEffector.R_e = (Rz(0)*Ry(0)*Rx(0)); % R^B_E

% Max propeller speeds in Hz
config.uavParams.minPropSpeed = -220;
config.uavParams.maxPropSpeed = 220;

config.gravity = 9.81;

%% Controller parameters
% PID gains for Genom3
config.conrtolParams.gainsPID.Kp = diag([23;23;25]);
config.conrtolParams.gainsPID.Kv = diag([15;15;20]);
config.conrtolParams.gainsPID.KR = diag([300;300;330]);
config.conrtolParams.gainsPID.Kw = diag([80;80;80]);
config.conrtolParams.gainsPID.KI_p = diag([4;4;6]);

% config.conrtolParams.gainsPID.Kp = diag([40;40;50]);
% config.conrtolParams.gainsPID.Kv = diag([17;17;22]);
% config.conrtolParams.gainsPID.KR = diag([320;320;360]);
% config.conrtolParams.gainsPID.Kw = diag([80;80;80]);
% config.conrtolParams.gainsPID.KI_p = diag([6;6;10]);

% Emergency parameters
config.conrtolParams.emerg.descent = 3;
config.conrtolParams.emerg.dq = 1;
config.conrtolParams.emerg.dw = 1;
config.conrtolParams.emerg.dx = 0.1;
config.conrtolParams.emerg.dv = 0.3;

% Error saturation
config.conrtolParams.errSat.e_p = 0.2;
config.conrtolParams.errSat.e_v = 2;
config.conrtolParams.errSat.e_i = 0.1;
config.conrtolParams.errSat.e_qi = 0.0;

% Admittance parameters
%{
% Point contact: f = 3N, x = 0.1 m
config.conrtolParams.admitFilt.virtMass = diag([1.5;1.5;1.5;1.5;1.5;1.5]);
config.conrtolParams.admitFilt.virtDamp = diag([13.5;13.5;13.5;13.5;13.5;13.5]);
config.conrtolParams.admitFilt.virtStiff = diag([30;30;30;30;30;30]);
%}


% Free flight: overdamped
config.conrtolParams.admitFilt.virtMass = diag([1.5;1.5;1.5;1.5;1.5;1.5]);
config.conrtolParams.admitFilt.virtDamp = diag([40;40;40;40;40;40]);
config.conrtolParams.admitFilt.virtStiff = diag([100;100;100;100;100;100]);


%{
% Peg-in-hole: tau - 0.6Nm, theta = 15 deg, 10 degree misalignment
config.conrtolParams.admitFilt.virtMass = diag([1.5;1.5;1.5;0.5;0.5;0.5]);
config.conrtolParams.admitFilt.virtDamp = diag([15;15;15;2.14;2.14;2.14]);
config.conrtolParams.admitFilt.virtStiff = diag([35;35;35;2.29;2.29;2.29]);
%}

%{
% Sliding: f = 3N, x = 0.1 m
config.conrtolParams.admitFilt.virtMass = diag([1.5;1.5;1.5;1.5;1.5;1.5]);
config.conrtolParams.admitFilt.virtDamp = diag([13.5;13.5;13.5;13.5;13.5;13.5]);
config.conrtolParams.admitFilt.virtStiff = diag([30;30;30;30;30;30]);
%}

% Wrench estimation observer gain
config.conrtolParams.wrenchObs.gains = diag([12;15;15;25;25;25]);
config.conrtolParams.wrenchObs.cutOff = [10,10,10,10,10,10];
config.conrtolParams.wrenchObs.thres = [0;0;0;0;0;0];
%config.conrtolParams.wrenchObs.bias = [0;0;1.3;0;0;0]; % Bias when hovering R=eye(3)
config.conrtolParams.wrenchObs.bias = [0;0;0;0;0;0];

%% Maneuver parameters
config.trajParams.trajXMin = -1.5;
config.trajParams.trajXMax = 1.5;
config.trajParams.trajYMin = -1.5;
config.trajParams.trajYMax = 1.5;
config.trajParams.trajZMin = 0;
config.trajParams.trajZMax = 2;

config.trajParams.trajVMax = 1;
config.trajParams.trajAMax = 1;
config.trajParams.trajJMax = 5;
config.trajParams.trajSMax = 1;

config.trajParams.trajYawMin = -pi;
config.trajParams.trajYawMax = pi;

config.trajParams.trajWMax = 1;
config.trajParams.trajDwMax = 1;
config.trajParams.trajDdwMax = 2;
config.trajParams.trajDddwMax = 2;

config.trajParams.radius = 1;
config.trajParams.frequency = 0.2;

%% Free flights

%% Cable - Free flight
% cableOffset = [-0.157;-0.623;0.88];
% config.trajParams.timePoints = [0,3,6,9,12,15];
% config.trajParams.wayPoints = [0, cableOffset(1), cableOffset(1),     cableOffset(1),     cableOffset(1), 0;  
%                                0, cableOffset(2), cableOffset(2),     cableOffset(2),     cableOffset(2), 0;
%                                0, cableOffset(3)+0.5, cableOffset(3)+0.5, cableOffset(3)+0.5, cableOffset(3)+0.5, 0];
% tilt = deg2rad(45);
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = eye(3);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(tilt);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(tilt);
% config.trajParams.rotPoints(:,:,5) = eye(3);
% config.trajParams.rotPoints(:,:,6) = eye(3);

% cableOffset = [-0.157;-0.623;0.88];
% config.trajParams.timePoints = [0,3,6,9,12,15];
% config.trajParams.wayPoints = [0, cableOffset(1), cableOffset(1),     cableOffset(1),     cableOffset(1), 0;  
%                                0, cableOffset(2), cableOffset(2),     cableOffset(2),     cableOffset(2), 0;
%                                0, cableOffset(3)+0.5, cableOffset(3)+0.5, cableOffset(3)+0.5, cableOffset(3)+0.5, 0];
% tilt = deg2rad(350);
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = eye(3);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(tilt);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(tilt);
% config.trajParams.rotPoints(:,:,5) = eye(3);
% config.trajParams.rotPoints(:,:,6) = eye(3);


%{
% Free flight - Translation
%config.trajParams.timePoints = [0,3,6,9,12];
config.trajParams.timePoints = [0,3,5,7,10];
%config.trajParams.timePoints = [0,3,4,5,8];
config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
                               0,  0,   0,   0,   0;
                               0,  1.2, 1.2, 1.2, 0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);

%}

% Free flight - Rotation
%{
tilt = deg2rad(45);
config.trajParams.timePoints = [0,3,6,9,12];
%config.trajParams.timePoints = [0,3,5,7,10];
%config.trajParams.timePoints = [0,3,4,5,8];
config.trajParams.wayPoints = [0,  0,   0,   0,   0; 
                               0,  0,   0,   0,   0;
                               0,  1.2, 1.2, 1.2, 0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(tilt)*Rx(0);
config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);
%}


% Free flight - Half Rotation
% tilt = deg2rad(180);
% config.trajParams.timePoints = [0,3,8,12,15];
% config.trajParams.wayPoints = [0,  0,   0,   0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(tilt)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);



% Free flight - Combined translation and rotation 1
config.trajParams.timePoints = [0,3,6,9,12];
config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
                               0,  0,   0,   0,   0;
                               0,  1.2, 1.2, 1.2, 0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Rz(pi/3)*Ry(pi/4);
config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);




% %Free flight - Combined translation and rotation 2
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Ry(pi/6)*Rx(pi/4);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);



% Free flight - Combined translation and rotation 3
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Ry(-pi/3)*Rx(pi/3);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);

%Free flight - Combined translation and rotation 4
% config.trajParams.timePoints = [0,3,6,9,12,15];
% config.trajParams.wayPoints = [0,  0,   0.5, 0, 0,   0; 
%                                0,  0,   0,   0, 0,   0;
%                                0,  1.2, 1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(pi/6);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(pi/3);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,6) = Rz(0)*Ry(0)*Rx(0);


% Free flight - Tilted translation 
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(pi/6);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(pi/6);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(pi/6);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);


% Free flight - Tilted translation 90 deg x
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.5, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);


% Free flight - Tilted translation 90 deg y
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.5, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(pi/2)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(pi/2)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(pi/2)*Rx(0);


% Free flight - "Orbit + yaw tracking + tilt oscillation"
% r = 0.6;                 % raggio cerchio (m)
% z1 = 1.0;                % quota di crociera (m)
% tiltP = deg2rad(15);     % pitch tilt (rad)
% tiltR = deg2rad(10);     % roll tilt (rad)
% 
% config.trajParams.timePoints = [ ...
%     0,  2.5,  5.0,  7.5, 10.0, 12.5, 15.0, 17.0, 20.0];
% config.trajParams.wayPoints = [ ...
%      0,    0,     r,     0,    -r,     0,     r,     0,     0;   % x
%      0,    0,     0,     r,     0,    -r,     0,     0,     0;   % y
%      0,   z1,    z1,    z1,    z1,    z1,    z1,   z1,     0];  % z
% 
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(deg2rad(0)) * Ry(0)        * Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(deg2rad(0))   * Ry(+tiltP) * Rx(-tiltR);
% config.trajParams.rotPoints(:,:,4) = Rz(deg2rad(90))  * Ry(-tiltP) * Rx(+tiltR);
% config.trajParams.rotPoints(:,:,5) = Rz(deg2rad(180)) * Ry(+tiltP) * Rx(-tiltR);
% config.trajParams.rotPoints(:,:,6) = Rz(deg2rad(270)) * Ry(-tiltP) * Rx(+tiltR);
% config.trajParams.rotPoints(:,:,7) = Rz(deg2rad(0))   * Ry(+tiltP) * Rx(0);
% config.trajParams.rotPoints(:,:,8) = Rz(deg2rad(135)) * Ry(-tiltP) * Rx(-tiltR);
% config.trajParams.rotPoints(:,:,9) = eye(3);


%% Point contact
%{
% Point contact straight surface
surfaceMiddle = [0.637; -0.827; 1.429];
d = 0.1;
config.trajParams.timePoints = [0,5,8,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,58];
PC = repmat([surfaceMiddle(1); surfaceMiddle(2)-d; surfaceMiddle(3)],1,13);
config.trajParams.wayPoints = [0, surfaceMiddle(1),     surfaceMiddle(1), PC(1,:), surfaceMiddle(1), surfaceMiddle(1),     0;  
                               0, surfaceMiddle(2)+0.3, surfaceMiddle(2), PC(2,:), surfaceMiddle(2), surfaceMiddle(2)+0.3, 0;
                               0, surfaceMiddle(3),     surfaceMiddle(3), PC(3,:), surfaceMiddle(3), surfaceMiddle(3),     0];

config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,4) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,5) = Rz(pi)*Ry(0)*Rx(deg2rad(20));
config.trajParams.rotPoints(:,:,6) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,7) = Rz(pi)*Ry(0)*Rx(-deg2rad(20));
config.trajParams.rotPoints(:,:,8) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,9) = Rz(pi)*Ry(deg2rad(20))*Rx(0);
config.trajParams.rotPoints(:,:,10) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,11) = Rz(pi)*Ry(-deg2rad(20))*Rx(0);
config.trajParams.rotPoints(:,:,12) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,13) = Rz(pi+deg2rad(20))*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,14) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,15) = Rz(pi+deg2rad(-20))*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,16) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,17) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,18) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,19) = eye(3);
%}

%{
% Point contact tilted surface 30 degree down
surfaceMiddle = [0.639; -0.848; 1.365];
d = 0.07;
config.trajParams.timePoints = [0,5,8,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,58];
PC = repmat([surfaceMiddle(1); surfaceMiddle(2)-d; surfaceMiddle(3)+d],1,13);
config.trajParams.wayPoints = [0, surfaceMiddle(1),      surfaceMiddle(1), PC(1,:), surfaceMiddle(1), surfaceMiddle(1),      0;  
                               0, surfaceMiddle(2)+0.15, surfaceMiddle(2), PC(2,:), surfaceMiddle(2), surfaceMiddle(2)+0.15, 0;
                               0, surfaceMiddle(3)-0.15, surfaceMiddle(3), PC(3,:), surfaceMiddle(3), surfaceMiddle(3)-0.15, 0];

config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,3) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,4) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,5) = Rz(pi)*Ry(0)*Rx(deg2rad(20)+deg2rad(20));
config.trajParams.rotPoints(:,:,6) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,7) = Rz(pi)*Ry(0)*Rx(deg2rad(30)-deg2rad(20));
config.trajParams.rotPoints(:,:,8) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,9) = Rz(pi)*Rx(deg2rad(30))*Ry(deg2rad(20));
config.trajParams.rotPoints(:,:,10) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,11) = Rz(pi)*Rx(deg2rad(30))*Ry(-deg2rad(20));
config.trajParams.rotPoints(:,:,12) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,13) = Rz(pi+deg2rad(20))*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,14) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,15) = Rz(pi+deg2rad(-20))*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,16) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,17) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,18) = Rz(pi)*Ry(0)*Rx(deg2rad(30));
config.trajParams.rotPoints(:,:,19) = eye(3);
%}

%% Sliding
%{
% Sliding - wind generator
%tunnelCentre = [1;1;1];
%tunnelCentre = [0.364;0.965+0.55;0.729];
%tunnelCentre = [0.338;0.876+0.55;0.711];
%tunnelCentre = [0.3959;0.877+0.55;0.730];
tunnelCentre = [0.0261;0.9452+0.55;0.7648];
tunnelRad = 0.55-0.03;
th = deg2rad(45);
initPoint = tunnelCentre + [0; -tunnelRad*cos(0); tunnelRad*sin(0)];
finPoint = tunnelCentre + [0; -tunnelRad*cos(th); tunnelRad*sin(th)];
approachPoint = initPoint - [0;0.1;0];

zAxis = [0; tunnelRad*sin(0); tunnelRad*cos(0)]/vecnorm([0; tunnelRad*sin(0); tunnelRad*cos(0)]);
yAxis = -[0; -tunnelRad*cos(0); tunnelRad*sin(0)]/vecnorm([0; -tunnelRad*cos(0); tunnelRad*sin(0)]);
xAxis = cross(yAxis,zAxis);
inOr = [xAxis,yAxis,zAxis];

zAxis = [0; tunnelRad*sin(th); tunnelRad*cos(th)]/vecnorm([0; tunnelRad*sin(th); tunnelRad*cos(th)]);
yAxis = -[0; -tunnelRad*cos(th); tunnelRad*sin(th)]/vecnorm([0; -tunnelRad*cos(th); tunnelRad*sin(th)]);
xAxis = cross(yAxis,zAxis);
finOr = [xAxis,yAxis,zAxis];

config.trajParams.timePoints = [0,5,10,25,40,45,50];
config.trajParams.wayPoints = [zeros(3,1), approachPoint, initPoint, finPoint, initPoint, approachPoint, zeros(3,1)];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = inOr;
config.trajParams.rotPoints(:,:,3) = inOr;
config.trajParams.rotPoints(:,:,4) = finOr;
config.trajParams.rotPoints(:,:,5) = inOr;
config.trajParams.rotPoints(:,:,6) = inOr;
config.trajParams.rotPoints(:,:,7) = eye(3);
%}

%% Peg in hole
%{
% Peg-in-hole staight - cable
cableOffset = [-0.157;-0.623;0.88];
%hole = [-0.124,-1.498,1.299];
hole = [-0.212,-1.424,1.434];
config.trajParams.timePoints = [0,3,6,9,12,15,18];
config.trajParams.wayPoints = [0, cableOffset(1), hole(1),  hole(1),      hole(1),  cableOffset(1), 0;  
                               0, cableOffset(2), hole(2),  hole(2)-0.2,  hole(2),  cableOffset(2), 0;
                               0, hole(3),        hole(3),  hole(3),      hole(3),  hole(3),        0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,4) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,5) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,6) = Rz(pi)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,7) = eye(3);
%}

%{
% Peg-in-hole tilted - cable
tilt = deg2rad(15);
cableOffset = [-0.157;-0.623;0.84];
%hole = [-0.124,-1.498,1.299];
%hole = [-0.187-0.02,-1.417,1.433+0.025];
hole = [-0.093, -1.419, 1.438-0.01];
config.trajParams.timePoints = [0,3,6,9,12,15,18,21];
config.trajParams.wayPoints = [0, cableOffset(1), hole(1),  hole(1),      hole(1),      hole(1),  cableOffset(1), 0;  
                               0, cableOffset(2), hole(2),  hole(2)-0.17, hole(2)-0.17, hole(2),  cableOffset(2), 0;
                               0, hole(3),        hole(3),  hole(3),      hole(3),      hole(3),  hole(3),        0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(pi)*Rx(-tilt);
config.trajParams.rotPoints(:,:,3) = Rz(pi)*Rx(-tilt);
config.trajParams.rotPoints(:,:,4) = Rz(pi)*Rx(-tilt);
config.trajParams.rotPoints(:,:,5) = Rz(pi)*Rx(-tilt);
config.trajParams.rotPoints(:,:,6) = Rz(pi)*Rx(-tilt);
config.trajParams.rotPoints(:,:,7) = Rz(pi)*Rx(-tilt);
config.trajParams.rotPoints(:,:,8) = eye(3);
%}

%{
% Peg-in-hole tilted - sim
tilt = deg2rad(30);
%hole = [-0.124,-1.498,1.299];
hole = [1,0,1];
config.trajParams.timePoints = [0,3,6,9,12,15,18];
config.trajParams.wayPoints = [0, 0,       hole(1),  hole(1)+0.5, hole(1),  0,       0;  
                               0, 0,       hole(2),  hole(2),      hole(2),  0,       0;
                               0, hole(3), hole(3),  hole(3),      hole(3),  hole(3), 0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(-pi/2)*Rx(-tilt);
config.trajParams.rotPoints(:,:,3) = Rz(-pi/2)*Rx(-tilt);
config.trajParams.rotPoints(:,:,4) = Rz(-pi/2)*Rx(-tilt);
config.trajParams.rotPoints(:,:,5) = Rz(-pi/2)*Rx(-tilt);
config.trajParams.rotPoints(:,:,6) = Rz(-pi/2)*Rx(-tilt);
config.trajParams.rotPoints(:,:,7) = eye(3);
%}

%% Wrench map optimization data gathering
%{
% No cable - 5 rots
load(strcat(baseFolder,"Rots.mat"))

config.trajParams.wayPoints = [zeros(3,1), [0.3;-0.2;1.3], repmat([0.3;-0.2;1.3],1,numel(Rots(1,1,:))*2), [0.3;-0.2;1.3], zeros(3,1)];
config.trajParams.timePoints = [0, 3, (cumsum(repmat([3 5], 1, numel(Rots(1,1,:)))) + 3), 46, 49];

config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = eye(3);
config.trajParams.rotPoints(:,:,3) = Rots(:,:,1);
config.trajParams.rotPoints(:,:,4) = Rots(:,:,1);
config.trajParams.rotPoints(:,:,5) = Rots(:,:,2);
config.trajParams.rotPoints(:,:,6) = Rots(:,:,2);
config.trajParams.rotPoints(:,:,7) = Rots(:,:,3);
config.trajParams.rotPoints(:,:,8) = Rots(:,:,3);
config.trajParams.rotPoints(:,:,9) = Rots(:,:,4);
config.trajParams.rotPoints(:,:,10) = Rots(:,:,4);
config.trajParams.rotPoints(:,:,11) = Rots(:,:,5);
config.trajParams.rotPoints(:,:,12) = Rots(:,:,5);
config.trajParams.rotPoints(:,:,13) = eye(3);
config.trajParams.rotPoints(:,:,14) = eye(3);
%}

%{
% Cable - 5 rots
load(strcat(baseFolder,"Rots.mat"))
cableOffset = [-0.17;-0.63;0.88];
config.trajParams.wayPoints = [zeros(3,1), [-0.17;-0.63;1.3], repmat([-0.17;-0.63;1.3],1,numel(Rots(1,1,:))*2), [-0.17;-0.63;1.3], zeros(3,1)];
config.trajParams.timePoints = [0, 3, (cumsum(repmat([3 5], 1, numel(Rots(1,1,:)))) + 3), 46, 49];

config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = eye(3);
config.trajParams.rotPoints(:,:,3) = Rots(:,:,1);
config.trajParams.rotPoints(:,:,4) = Rots(:,:,1);
config.trajParams.rotPoints(:,:,5) = Rots(:,:,2);
config.trajParams.rotPoints(:,:,6) = Rots(:,:,2);
config.trajParams.rotPoints(:,:,7) = Rots(:,:,3);
config.trajParams.rotPoints(:,:,8) = Rots(:,:,3);
config.trajParams.rotPoints(:,:,9) = Rots(:,:,4);
config.trajParams.rotPoints(:,:,10) = Rots(:,:,4);
config.trajParams.rotPoints(:,:,11) = Rots(:,:,5);
config.trajParams.rotPoints(:,:,12) = Rots(:,:,5);
config.trajParams.rotPoints(:,:,13) = eye(3);
config.trajParams.rotPoints(:,:,14) = eye(3);
%}


%% Demo flight

% First half rotation then sliding
% Half rotation
% tilt = deg2rad(180);
% config.trajParams.timePoints = [0,3,8,12,15];
% config.trajParams.wayPoints = [0,  0,   0,   0, 0; 
%                                0,  0,   0,   0, 0;
%                                0,  1.2, 1.2, 1.2, 1.2];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(tilt)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);
% 
% % Sliding 
% tunnelCentre = [0.0261;0.9452+0.55;0.7648];
% tunnelRad = 0.55-0.05;
% th = deg2rad(45);
% initPoint = tunnelCentre + [0; -tunnelRad*cos(0); tunnelRad*sin(0)];
% finPoint = tunnelCentre + [0; -tunnelRad*cos(th); tunnelRad*sin(th)];
% approachPoint = initPoint - [0;0.1;0];
% 
% zAxis = [0; tunnelRad*sin(0); tunnelRad*cos(0)]/vecnorm([0; tunnelRad*sin(0); tunnelRad*cos(0)]);
% yAxis = -[0; -tunnelRad*cos(0); tunnelRad*sin(0)]/vecnorm([0; -tunnelRad*cos(0); tunnelRad*sin(0)]);
% xAxis = cross(yAxis,zAxis);
% inOr = [xAxis,yAxis,zAxis];
% 
% zAxis = [0; tunnelRad*sin(th); tunnelRad*cos(th)]/vecnorm([0; tunnelRad*sin(th); tunnelRad*cos(th)]);
% yAxis = -[0; -tunnelRad*cos(th); tunnelRad*sin(th)]/vecnorm([0; -tunnelRad*cos(th); tunnelRad*sin(th)]);
% xAxis = cross(yAxis,zAxis);
% finOr = [xAxis,yAxis,zAxis];
% 
% config.trajParams.timePoints = [config.trajParams.timePoints,[20,25,40,55,60,63]]; 
% config.trajParams.wayPoints = [config.trajParams.wayPoints, [approachPoint, initPoint, finPoint, initPoint, approachPoint, zeros(3,1)]];
% config.trajParams.rotPoints(:,:,6) = inOr;
% config.trajParams.rotPoints(:,:,7) = inOr;
% config.trajParams.rotPoints(:,:,8) = finOr;
% config.trajParams.rotPoints(:,:,9) = inOr;
% config.trajParams.rotPoints(:,:,10) = inOr;
% config.trajParams.rotPoints(:,:,11) = eye(3);

config.trajParams.initVel = zeros(3,numel(config.trajParams.timePoints));
config.trajParams.initAcc = zeros(3,numel(config.trajParams.timePoints)); 
config.trajParams.initOm = zeros(3,numel(config.trajParams.timePoints));
config.trajParams.numSamples = 40000;

%% Simulink variables
% Note: Simulink cannot handle lower time step
config.simVar.timeStep = 0.004; 

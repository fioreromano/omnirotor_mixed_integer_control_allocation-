%% UAV params
config.uavParams.np = 8;
config.uavParams.mass = 1.5309; 
config.uavParams.com = [0, 0, 0]';
config.uavParams.c_f = 1.5e-4; 
config.uavParams.c_t = 2.2e-6;
config.uavParams.inertia = diag([3.332e-2;1.138e-2;3.332e-2]);
config.uavParams.L = 0.2165/sqrt(3);

% End effector parameters
config.tool.l = 0.476;
config.tool.r = 0.04;
config.tool.p_e = [0;config.tool.l;0]; % EE pos
config.tool.R_e = (Rz(0)*Ry(0)*Rx(0)); % R^B_E
config.tool.mass = 0.0941;
config.tool.com = [0, config.tool.l , 0]';

config.system.com = (config.uavParams.mass*config.uavParams.com + config.tool.mass*config.tool.com)/(config.uavParams.mass + config.tool.mass);
config.system.mass = config.uavParams.mass + config.tool.mass;

% Max propeller speeds in Hz
config.uavParams.minPropSpeed = -300;
config.uavParams.maxPropSpeed = 300;

config.gravity = 9.81;

cf = config.uavParams.c_f;
ct = config.uavParams.c_t;
tau = config.uavParams.c_t/config.uavParams.c_f;

%% Control params
config.conrtolParams.gainsPID.Kp1 = [70, 70, 70];
config.conrtolParams.gainsPID.Kp2 = [100, 100, 100];
config.conrtolParams.gainsPID.Kp3 = [80, 80, 80];
config.conrtolParams.gainsPID.KR1 = [120, 120, 120];
config.conrtolParams.gainsPID.KR2 = [250, 250, 300];
config.conrtolParams.gainsPID.KR3 = [100, 100, 120];

%% Trajectory params

% Free flight - Translation
%config.trajParams.timePoints = [0,3,6,9,12];
%config.trajParams.timePoints = [0,3,5,7,10];
%config.trajParams.timePoints = [0,3,4,5,8];

% config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);


% Free flight - Rotation

% tilt = deg2rad(45);
% config.trajParams.timePoints = [0,3,6,9,12];
% %config.trajParams.timePoints = [0,3,5,7,10];
% %config.trajParams.timePoints = [0,3,4,5,8];
% config.trajParams.wayPoints = [0,  0,   0,   0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(tilt)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);



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
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(pi/3)*Ry(pi/4);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);



%Free flight - Combined translation and rotation 2
config.trajParams.timePoints = [0,3,6,9,12];
config.trajParams.wayPoints = [0,  0,   0.5, 0,   0; 
                               0,  0,   0,   0,   0;
                               0,  1.2, 1.2, 1.2, 0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Ry(pi/6)*Rx(pi/4);
config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);



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
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(30);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(60);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(90);
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

% Free flight - Tilted translation 
% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.5, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(pi/2);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);

% config.trajParams.timePoints = [0,3,6,9,12];
% config.trajParams.wayPoints = [0,  0,   0, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.5, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(pi/2)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(pi/2)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(pi/2)*Rx(0);


config.trajParams.initVel = zeros(3,numel(config.trajParams.timePoints));
config.trajParams.initAcc = zeros(3,numel(config.trajParams.timePoints)); 
config.trajParams.initOm = zeros(3,numel(config.trajParams.timePoints));
config.trajParams.numSamples = 1000;

% config.trajParams.wayPoints(:,1) = posOffset+liftOff;
% config.trajParams.wayPoints(:,end) = posOffset+liftOff;
[pdes, vdes, ades, tdes] = minJerkPloyTraj(config.trajParams.wayPoints, ...
                                           config.trajParams.initVel, ...
                                           config.trajParams.initAcc, ...
                                           config.trajParams.timePoints, ...
                                           config.trajParams.numSamples);

% config.trajParams.rotPoints(:,:,1) = R_in;
% config.trajParams.rotPoints(:,:,end) = R_in;
[Rdes, omd, domd, tdes] = minAaccPolyTraj(config.trajParams.rotPoints, ...
                                    config.trajParams.initOm, ...
                                    config.trajParams.timePoints, ...
                                    config.trajParams.numSamples);

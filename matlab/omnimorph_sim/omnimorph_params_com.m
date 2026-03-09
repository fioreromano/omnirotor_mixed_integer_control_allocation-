%% UAV parameters
config.uavParams.mass = 1.625; % with peripherals and battery
config.uavParams.c_f = 1.5e-4; 
config.uavParams.c_t = 2.2e-6;
config.uavParams.inertia = diag([3.332e-2;1.138e-2;3.332e-2]);

% CoM_Mode = 1 -> we DO account for the shif in the control allocation matrix
% CoM_Mode = 2 -> we DO account for the shif in the control allocation
% matrix but with no shif optimal orientation
% CoM_Mode = 3 -> we DO NOT account for the shif in the control allocation
% matrix and with no shif optimal orientation

if CoM_Mode == 1
    % Shifted Optimal Orientations
    if CoM_choice == 1
        n = [ -0.5812   0.5937  -0.5538   0.5298  -0.5879   0.6310  -0.5771   0.5587;
              -0.2022  -0.1748   0.8023   0.8048   0.7854   0.7595  -0.2186  -0.2499;
              0.7883   0.7854   0.2230   0.2676   0.1938   0.1578   0.7869   0.7908];

    elseif CoM_choice == 2
        n = [   -0.5941    0.5783   -0.5327    0.5489   -0.6272    0.5937   -0.5597    0.5784;
                -0.1779   -0.2029    0.8057    0.8046    0.7613    0.7814   -0.2471   -0.2172;
                 0.7844    0.7902    0.2591    0.2265    0.1646    0.1920    0.7910    0.7863 ];
        
    elseif CoM_choice == 3
        n = [ -0.1962   0.7557   0.7897  -0.2637   0.8039  -0.1617  -0.2206   0.7995;
              -0.5815   0.6396  -0.5774   0.5565  -0.5580   0.5980  -0.5780   0.5293;
               0.7895   0.1406   0.2073   0.7879   0.2060   0.7850   0.7857   0.2838 ];

    elseif CoM_choice == 4
        n = [ -0.7906   0.2395   0.2240  -0.7498   0.1877  -0.8123  -0.7948   0.1770;
               0.5858  -0.5518   0.5902  -0.6408   0.5738  -0.5261   0.5613  -0.6056;
               0.1784   0.7988   0.7756   0.1651   0.7972   0.2516   0.2306   0.7758 ];

    elseif CoM_choice == 5
        n = [ -0.7908   0.2212   0.2302  -0.7447   0.1733  -0.8184  -0.7879   0.1832;
               0.5904  -0.5551   0.5942  -0.6439   0.5789  -0.5261   0.5679  -0.6078;
               0.1612   0.8018   0.7707   0.1752   0.7968   0.2310   0.2381   0.7727 ];

    elseif CoM_choice == 6
        n = [ -0.1413   0.7473   0.7864  -0.2812   0.8069  -0.1179  -0.2472   0.7804;
              -0.5956   0.6592  -0.5764   0.5642  -0.5678   0.6178  -0.5816   0.5419;
               0.7907   0.0831   0.2221   0.7763   0.1625   0.7775   0.7750   0.3119 ];
    end  

elseif CoM_Mode == 2 || CoM_Mode == 3
    % No shift Optimal Orientation -> standard one
    n = [ -0.5812   0.5937  -0.5538   0.5298  -0.5879   0.6310  -0.5771   0.5587;
          -0.2022  -0.1748   0.8023   0.8048   0.7854   0.7595  -0.2186  -0.2499;
           0.7883   0.7854   0.2230   0.2676   0.1938   0.1578   0.7869   0.7908];
end

nx = n(1,:).'; ny = n(2,:).'; nz = n(3,:).';
config.uavParams.propTiltPitch = atan2(nx, nz);  
config.uavParams.propTiltRoll  = -asin(ny); 

% Nominal wrench map
config.uavParams.wrenchMap = double(myWrenchMapCoM(config.uavParams.c_t, ...
                                                config.uavParams.c_f , ...
                                                n, ...
                                                0.2165/sqrt(3), ...
                                                config.uavParams.CoM, ...
                                                CoM_Mode));

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
config.uavParams.minPropSpeed = -300;
config.uavParams.maxPropSpeed = 300;

config.gravity = 9.81;

%% Controller parameters
% PID gains for Genom3
config.conrtolParams.gainsPID.Kp = diag([23;23;25]);
config.conrtolParams.gainsPID.Kv = diag([15;15;20]);
config.conrtolParams.gainsPID.KR = diag([300;300;330]);
config.conrtolParams.gainsPID.Kw = diag([80;80;80]);
config.conrtolParams.gainsPID.KI_p = diag([4;4;6]);

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

% Free flight - Hovering
% config.trajParams.timePoints = [0,3,5,7,10];
% config.trajParams.wayPoints = [0,  0,   0, 0,   0; 
%                                0,  0,   0,   0,   0;
%                                0,  1.2, 1.2, 1.2, 0];
% config.trajParams.rotPoints(:,:,1) = eye(3);
% config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(0);
% config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(0);

% Free flight - 360° Hovering
config.trajParams.timePoints = [0,4,7,10,13,16,19,21,24,27,30, 33, 36, 39, 42];
config.trajParams.wayPoints = [0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0; 
                               0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0;
                               0,  1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 0];
config.trajParams.rotPoints(:,:,1) = eye(3);
config.trajParams.rotPoints(:,:,2) = Rz(0)*Ry(0)*Rx(0);
config.trajParams.rotPoints(:,:,3) = Rz(0)*Ry(0)*Rx(pi/6);
config.trajParams.rotPoints(:,:,4) = Rz(0)*Ry(0)*Rx(pi/3);
config.trajParams.rotPoints(:,:,5) = Rz(0)*Ry(0)*Rx(pi/2);
config.trajParams.rotPoints(:,:,6) = Rz(0)*Ry(0)*Rx(4*pi/6);
config.trajParams.rotPoints(:,:,7) = Rz(0)*Ry(0)*Rx(5*pi/6);
config.trajParams.rotPoints(:,:,8) = Rz(0)*Ry(0)*Rx(pi);
config.trajParams.rotPoints(:,:,9) = Rz(0)*Ry(0)*Rx(7*pi/6);
config.trajParams.rotPoints(:,:,10) = Rz(0)*Ry(0)*Rx(4*pi/3);
config.trajParams.rotPoints(:,:,11) = Rz(0)*Ry(0)*Rx(3*pi/2);
config.trajParams.rotPoints(:,:,12) = Rz(0)*Ry(0)*Rx(5*pi/3);
config.trajParams.rotPoints(:,:,13) = Rz(0)*Ry(0)*Rx(11*pi/6);
config.trajParams.rotPoints(:,:,14) = Rz(0)*Ry(0)*Rx(2*pi);
config.trajParams.rotPoints(:,:,15) = Rz(0)*Ry(0)*Rx(2*pi);


config.trajParams.initVel = zeros(3,numel(config.trajParams.timePoints));
config.trajParams.initAcc = zeros(3,numel(config.trajParams.timePoints)); 
config.trajParams.initOm = zeros(3,numel(config.trajParams.timePoints));
config.trajParams.numSamples = 40000;

%% Simulink variables
% Note: Simulink cannot handle lower time step
config.simVar.timeStep = 0.004; 

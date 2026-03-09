close all
clear
clc

%% User-specific variables
localUser = 'fiore'; % local linux username
raspiUser = 'fiore'; % Raspi username
remoteIP = '192.168.0.122'; % Raspi IP
dataPath = '/home/fiore/MTP/data/CoM_Shift/360Hov/'; % where to save logged data locally

remoteLogPath = '/home/fiore/MTP/logs/'; % location of logs on Raspi
pocolibsPath = '/home/fiore/pocolibs/omni_start_output.log'; % location of genom3 log on Raspi

trajPath = '/home/fiore/MTP/trajectories/'; % where to save trajectory locally
remoteTrajPath = '/home/fiore/MTP/trajectories/'; % where to save trajectory on remote Raspi

develPath = '/home/fiore/devel/lib/genom/pocolibs/plugins/'; % custom genom3 development path
remoteDevelPath = '/home/fiore/devel/lib/genom/pocolibs/plugins/';

%% Add matlab paths and genomix plugins
openRobBase =  strcat('/home/',localUser,'/openrobots/');
addpath('./lib/')
addpath('./lib/omnimorph-helper/')
addpath('./lib/math-helper/')
addpath('./sim/')
addpath('./plot/')
addpath('./metrics/')
addpath('./wMap_identification/')

addpath(strcat(openRobBase,'lib/matlab/simulink/genomix/'))
addpath(strcat(openRobBase,'lib/matlab/simulink/'))
addpath(strcat(openRobBase,'lib/matlab/'))

%% Simulation or experiment
modes = ["Simulation", "Real experiment"];
% fprintf('Select a mode: \n');
% fprintf('1. Simulation in Gazebo \n');
% fprintf('2. Real-life experiment \n');
% experimentChoice = input('Enter the number corresponding to your choice (1-2): ');
% 
% while ~ismember(experimentChoice, [1, 2])
%     disp('Invalid input. Please enter 1 or 2.');
%     experimentChoice = input('Enter the number corresponding to your choice (1-2): ');
% end
experimentChoice = 1;
experimentFlag = (experimentChoice == 2);
% fprintf('\n');

%% Select controller
controllers = ["Simulink", "Omnimorph-genom3 with physical interaction", "Omnimorph-genom3 without physical interaction", "Omnimorph-genom3 with miqp control"];
% fprintf('Select a controller: \n');
% fprintf('1. Simulink \n');
% fprintf('2. Omnimorph-genom3 with physical interaction \n');
% fprintf('3. Omnimorph-genom3 without physical interaction \n');
% fprintf('4. Omnimorph-genom3 with MIQP control \n');
% controllerChoice = input('Enter the number corresponding to your choice (1-4): ');
% 
% while ~ismember(controllerChoice, [1, 2, 3, 4])
%     fprintf('Invalid input. Please enter 1, 2, 3 or 4. \n');
%     controllerChoice = input('Enter the number corresponding to your choice (1-4): ');
% end
% fprintf('\n');
controllerChoice = 3;

%% Select CoM handling mode
CoM_Modes = [ ...
    "Shifted optimal orientation (allocation updated)", ...
    "Nominal orientation (allocation updated with CoM shift)", ...
    "Nominal orientation (allocation NOT updated - Scenario C)" ...
];

fprintf('Select CoM handling mode: \n');
fprintf('1. Shifted optimal orientation (Scenario A)\n');
fprintf('2. Nominal orientation with updated allocation (Scenario B)\n');
fprintf('3. Nominal orientation without allocation update (Scenario C)\n');

CoM_Mode = input('Enter the number corresponding to your choice (1-3): ');

while ~ismember(CoM_Mode, [1, 2, 3])
    fprintf('Invalid input. Please enter 1, 2 or 3.\n');
    CoM_Mode = input('Enter the number corresponding to your choice (1-3): ');
end

%% Select CoM shift magnitude

CoM_Shifts = [ ...
    0   0   0   0   0   0;
    0   0.025   0.062   0.125  0.187  0.250;
    0   0   0   0   0   0 ];

fprintf('Select CoM shift magnitude:\n');
fprintf('1. 0.0\n');
fprintf('2. 0.025\n');
fprintf('3. 0.062\n');
fprintf('4. 0.125\n');
fprintf('5. 0.1875\n');
fprintf('6. 0.250\n');

CoM_choice = input('Enter the number corresponding to your choice (1-6): ');

while ~ismember(CoM_choice, 1:6)
    fprintf('Invalid input. Please enter a number between 1 and 6.\n');
    CoM_choice = input('Enter the number corresponding to your choice (1-6): ');
end

config.uavParams.CoM = CoM_Shifts(:, CoM_choice);

%% Ask wrench map mode
% if (controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4)
    wrenchMapModes = ["Nominal/modelled", "Piece-wise linear (optimized)"];
%     fprintf('Select wrench map mode: \n');
%     fprintf('1. Single Nominal wrench map \n');
%     fprintf('2. Piece-wise linear optimized wrench maps \n');
%     wMapModeChoice = input('Enter the number corresponding to your choice (1-2): ');
% 
%     while ~ismember(wMapModeChoice, [1, 2, 3])
%         fprintf('Invalid input. Please enter 1 or 2. \n');
%         wMapModeChoice = input('Enter the number corresponding to your choice (1-2): ');
%     end
%     fprintf('\n');
% end
wMapModeChoice = 1;

%% Ask if logging is desired
logChoice = lower(input('Do you want to enable logging? (y/n): ', 's'));

% Validate logging input
while ~ismember(logChoice, {'y', 'n'})
    fprintf('Invalid input. Please enter "yes" or "no".');
    logChoice = lower(input('Do you want to enable logging? (y/n): ', 's'));
end

% Initialize folder paths
logFlag = strcmp(logChoice, 'y');
if logFlag
    % Ask for session name
    sessionName = input('Enter a name for this session: ', 's');
    description = input('Enter a description: ', 's');

    % Generate timestamp
    timeStamp = string(datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss'));

    % Create folder structure
    baseFolder = strcat(dataPath,sessionName,'/');
    logFolder = strcat(baseFolder, 'logs/');
    figFolder = strcat(baseFolder, 'figures/');

    if isfolder(baseFolder)
        warning('The directory "%s" already exists. Logs may be overwritten.', baseFolder);
        proceed = lower(input('Do you want to continue? (y/n): ', 's'));
        if ~strcmp(proceed, 'y')
            error('Logging aborted by user.');
        end
    end

    mkdir(baseFolder);
    mkdir(logFolder);
    mkdir(figFolder);  

    fprintf('Logging enabled. Folders for files created in %s\n',baseFolder);

    % Load parameters in config structure. These parameters can be passed to
    % Simulink functions. 
    run('omnimorph_params_com')  

    % Make a metadata file
    metadataFile = fullfile(baseFolder, 'metadata.txt');
    fid = fopen(metadataFile, 'w');

    % Print all UAV and controller parameters
    if fid == -1
        warning('Could not create metadata file.');
    else
        fprintf(fid, 'Session Metadata\n');
        fprintf(fid, '----------------\n');
        fprintf(fid, 'Timestamp:      %s\n', timeStamp);
        fprintf(fid, 'Mode:           %s\n', modes(experimentChoice));
        fprintf(fid, 'Session Name:   %s\n', sessionName);
        fprintf(fid, 'Controller:     %s\n', controllers(controllerChoice));
        if (controllerChoice == 2 || controllerChoice == 3)
            fprintf(fid, 'Wrench map mode:%s\n', wrenchMapModes(wMapModeChoice));
        end
        fprintf(fid, 'Description:    %s\n', description);
        fprintf(fid, '\nUAV Parameters\n');
        fprintf(fid, '--------------\n');
        fprintf(fid, 'Mass:                 %.3f kg\n', config.uavParams.mass);
        fprintf(fid, 'c_f:                  %.1e\n', config.uavParams.c_f);
        fprintf(fid, 'c_t:                  %.1e\n', config.uavParams.c_t);
        fprintf(fid, 'Inertia (diag):       [%.2e, %.2e, %.2e] kg·m²\n', diag(config.uavParams.inertia));
        fprintf(fid, 'Propeller Tilt X:       %.2f deg\n', rad2deg(config.uavParams.propTiltRoll));
        fprintf(fid, 'Propeller Tilt Y:       %.2f deg\n', rad2deg(config.uavParams.propTiltPitch));
        fprintf(fid, 'Min Prop Speed (Hz):  %.1f\n', config.uavParams.minPropSpeed);
        fprintf(fid, 'Max Prop Speed (Hz):  %.1f\n', config.uavParams.maxPropSpeed);
        fprintf(fid, '\nNominal wrench map:\n');
        for i = 1:size(config.uavParams.wrenchMap, 1)
            fprintf(fid, '                      %s\n', sprintf('%.3e   ', config.uavParams.wrenchMap(i, :)));
        end
        fprintf(fid, '\nOptimized wrench maps:\n');
        for i = 1:size(config.uavParams.wMaps, 3)
            fprintf(fid, '    Region %.1f:\n', i);
            for j = 1:size(config.uavParams.wrenchMap, 1)
                fprintf(fid, '                      %s\n', sprintf('%.3e   ', config.uavParams.wMaps(j, :, i)));
            end
        end
        fprintf(fid, '\nEnd Effector Parameters\n');
        fprintf(fid, '-----------------------\n');
        fprintf(fid, 'p_e:                  [%s]\n', sprintf('%.2f ', config.uavParams.endEffector.p_e));
        fprintf(fid, 'R_e:\n');
        for i = 1:3
            fprintf(fid, '                      %s\n', sprintf('%.3f ', config.uavParams.endEffector.R_e(i, :)));
        end 
        fprintf(fid, '\nController Parameters\n');
        fprintf(fid, '---------------------\n');
        fprintf(fid, '\nPID Gains:\n');
        fprintf(fid, 'Kp:      [%s]\n', sprintf('%.1f ', diag(config.conrtolParams.gainsPID.Kp)));
        fprintf(fid, 'Kv:      [%s]\n', sprintf('%.1f ', diag(config.conrtolParams.gainsPID.Kv)));
        fprintf(fid, 'KR:      [%s]\n', sprintf('%.1f ', diag(config.conrtolParams.gainsPID.KR)));
        fprintf(fid, 'Kw:      [%s]\n', sprintf('%.1f ', diag(config.conrtolParams.gainsPID.Kw)));
        fprintf(fid, 'KI_p:    [%s]\n', sprintf('%.1f ', diag(config.conrtolParams.gainsPID.KI_p)));
        fprintf(fid, '\nEmergency Parameters:\n');
        fprintf(fid, 'Descent:           %.2f\n', config.conrtolParams.emerg.descent);
        fprintf(fid, 'dq (rot error):    %.2f\n', config.conrtolParams.emerg.dq);
        fprintf(fid, 'dw (ang vel err):  %.2f\n', config.conrtolParams.emerg.dw);
        fprintf(fid, 'dx (pos error):    %.2f\n', config.conrtolParams.emerg.dx);
        fprintf(fid, 'dv (vel error):    %.2f\n', config.conrtolParams.emerg.dv);
        fprintf(fid, '\nError Saturation:\n');
        fprintf(fid, 'e_p:   %.2f\n', config.conrtolParams.errSat.e_p);
        fprintf(fid, 'e_v:   %.2f\n', config.conrtolParams.errSat.e_v);
        fprintf(fid, 'e_i:   %.2f\n', config.conrtolParams.errSat.e_i);
        fprintf(fid, 'e_qi:  %.2f\n', config.conrtolParams.errSat.e_qi);
        fprintf(fid, '\nAdmittance filter parameters\n');
        fprintf(fid, '---------------------\n');        
        fprintf(fid, '\nVirtual Mass:\n');
        for i = 1:size(config.conrtolParams.admitFilt.virtMass,1)
            fprintf(fid, '                      %s\n', sprintf('%.2f ', config.conrtolParams.admitFilt.virtMass(i,:)));
        end
        fprintf(fid, '\nVirtual Damping:\n');
        for i = 1:size(config.conrtolParams.admitFilt.virtDamp,1)
            fprintf(fid, '                      %s\n', sprintf('%.2f ', config.conrtolParams.admitFilt.virtDamp(i,:)));
        end
        fprintf(fid, '\nVirtual Stiffness:\n');
        for i = 1:size(config.conrtolParams.admitFilt.virtStiff,1)
            fprintf(fid, '                      %s\n', sprintf('%.2f ', config.conrtolParams.admitFilt.virtStiff(i,:)));
        end
        fprintf(fid, '\nWrench Observer Gains:\n');
        fprintf(fid, '                     [%s]\n', sprintf('%.1f ', diag(config.conrtolParams.wrenchObs.gains))); 
        fprintf(fid, '\nLow pass filter cut-off:\n');
        fprintf(fid, '                     [%s]\n', sprintf('%.1f ', config.conrtolParams.wrenchObs.cutOff)); 
        fprintf(fid, '\nThresholds:\n');
        fprintf(fid, '                     [%s]\n', sprintf('%.1f ', config.conrtolParams.wrenchObs.thres)); 
        fprintf(fid, '\nBias:\n');
        fprintf(fid, '                     [%s]\n', sprintf('%.1f ', config.conrtolParams.wrenchObs.bias));         
        fclose(fid);
        fprintf('Metadata written to %s\n', metadataFile);
    end

    if experimentFlag
        logPath = remoteLogPath; % path for saving logs on remote raspi
    else
        logPath = logFolder;
    end
else
    % Load parameters in config structure. These parameters can be passed to
    % Simulink functions. 
    run('omnimorph_params_com')  

    fprintf('Logging disabled. No folders will be created. \n');
end
fprintf('\n');

%% Load genomix client
if experimentFlag
    host = remoteIP;
else
    host = 'localhost';
end

client = genomix.client(host);
pause(0.5)

%% Rotorcraft settings 
rotorcraft = client.load('rotorcraft');
pause(0.5);

usbPort = 'chimera-115';
if experimentFlag
    result = rotorcraft.connect(usbPort,0);
    checkResult(result.status, 'rotorcraft.connect to chimera', result.exception);

    load("calib_omni.mat")
    result = rotorcraft.set_imu_calibration(calibration);  
    checkResult(result.status, 'rotorcraft.set_imu_calibration', result.exception);
else
    result = rotorcraft.connect('/tmp/pty-omnimorph',0);
    checkResult(result.status, 'rotorcraft.connect', result.exception);
end

% Sensor rates
sensRate.imu = 1000;
sensRate.mag = 0;
sensRate.motor = 500;
sensRate.battery = 1;

result = rotorcraft.set_sensor_rate(sensRate.imu, ...
                                    sensRate.mag, ...
                                    sensRate.motor, ...
                                    sensRate.battery);
checkResult(result.status, 'rotorcraft.set_sensor_rate', result.exception);

% IMU Filters
imuFilter.gyro = [20; 20; 20];
imuFilter.acc = [5; 5; 5];
imuFilter.mag = [0;0;0];

result = rotorcraft.set_imu_filter(num2cell(imuFilter.gyro), ...
                                   num2cell(imuFilter.acc), ...
                                   num2cell(imuFilter.mag));
checkResult(result.status, 'rotorcraft.set_imu_filter', result.exception);

% Low-level Motor PIDs
motPID.Kp = 100;
motPID.Ki = 500;
motPID.Kd = 0;
motPID.Kf = 100;

for mot=1:1:8
   result = rotorcraft.set_pid(mot,motPID.Kp, ...
                               motPID.Ki, ...
                               motPID.Kd, ...
                               motPID.Kf);
   checkResult(result.status, 'rotorcraft.set_pid', result.exception);
   pause(0.1)
end

%% Dynamixel settings
dx_pitch  = client.load('dynamixel', '-i', 'dynamixel_pitch');
pause(0.5)
dx_roll  = client.load('dynamixel', '-i', 'dynamixel_roll');
pause(0.5)

if ~experimentFlag
    % NB: questi PTY devono corrispondere a quelli creati dai due dxsnim in Gazebo
    result = dx_pitch.connect('/tmp/pty-dynamixel-pitch');
    checkResult(result.status, 'dynamixel_pitch.connect', result.exception);

    result = dx_roll.connect('/tmp/pty-dynamixel-roll');
    checkResult(result.status, 'dynamixel_roll.connect', result.exception);

    p = config.uavParams.propTiltPitch(:);   % 8x1
    r = config.uavParams.propTiltRoll(:);    % 8x1

    result = dx_pitch.set_position(num2cell(p));
    checkResult(result.status, 'dynamixel_pitch.set_position', result.exception);

    pause(1.0)

    result = dx_roll.set_position(num2cell(r));
    checkResult(result.status, 'dynamixel_roll.set_position', result.exception);
end


%% Optitrack settings
optitrack = client.load('optitrack');
pause(0.5)

if experimentFlag
    opti.ip = '192.168.0.102';
    opti.cmdPort = '1510';
    opti.dataPort = '1511';
    opti.multicastAdr = '239.255.42.99';  
else
    opti.ip = 'localhost';
    opti.cmdPort = '1509';
    opti.dataPort = '0';
    opti.multicastAdr = '';    
end
result = optitrack.connect(opti.ip, opti.cmdPort, ...
                                   opti.multicastAdr, opti.dataPort);
checkResult(result.status, 'optitrack.connect', result.exception);

%% State estimation (POM) settings 
pom = client.load('pom');
pause(0.5)

result = pom.connect_port('measure/imu', 'rotorcraft/imu');
checkResult(result.status, 'pom.connect_port to rotorcraft', result.exception);

result = pom.connect_port('measure/mocap', 'optitrack/bodies/omni_morph');
checkResult(result.status, 'pom.connect_port to optitrack', result.exception);

% IMU offset
if experimentFlag
    imuOffset.x = 0;
    imuOffset.y = 0;
    imuOffset.z = 0.041; % omni_fixed, where the board is w.r.t center of vehicle          
else
    imuOffset.x = 0;
    imuOffset.y = 0;
    imuOffset.z = 0;               
end
result = pom.add_measurement('imu',imuOffset.x, ...
                                   imuOffset.y, ...
                                   imuOffset.z,0,0,0);
checkResult(result.status, 'pom.add_measurement', result.exception);

result = pom.add_measurement('mocap');
checkResult(result.status, 'pom.add_measurement', result.exception);

result = pom.set_history_length(0.3);
checkResult(result.status, 'pom.set_history_length', result.exception);

%% Omni genom3 controller
if controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4
    if experimentFlag
        omnimorph = client.load(strcat(remoteDevelPath,'omnimorph'));
    else
        omnimorph = client.load(strcat(develPath,'omnimorph'));
    end
        pause(0.5)

    result = omnimorph.connect_port('state', 'pom/frame/robot');
    checkResult(result.status, 'omnimorph.connect_port to POM', result.exception);

    result = omnimorph.connect_port('rotor_measure', 'rotorcraft/rotor_measure');
    checkResult(result.status, 'omnimorph.connect_port rotor measure to rotorcraft', result.exception);

    result = rotorcraft.connect_port('rotor_input', 'omnimorph/rotor_input');
    checkResult(result.status, 'omnimorph.connect_port to rotorcraft input', result.exception);

    result = omnimorph.set_geom(config.uavParams.mass, ...
                                num2cell(reshape(config.uavParams.wrenchMap',[48,1])), ...
                                num2cell(reshape(config.uavParams.inertia',[9,1])));              
    checkResult(result.status, 'omnimorph.set_geom', result.exception);

    wmaps = [];
    for i=1:10
        wmaps = [wmaps;reshape(config.uavParams.wMaps(:,:,i)',[48,1])];
    end

    if wMapModeChoice == 2
    	result = omnimorph.set_wmaps(config.uavParams.mass, ...
                                     num2cell(wmaps), ...
                                     num2cell(reshape(config.uavParams.inertia',[9,1])));              
        checkResult(result.status, 'omnimorph.set_wmaps', result.exception);
    end    

    result = omnimorph.set_wlimit(config.uavParams.minPropSpeed,config.uavParams.maxPropSpeed);
    checkResult(result.status, 'omnimorph.set_wlimit',result.exception);
    result = omnimorph.set_wlimit(config.uavParams.minPropSpeed,config.uavParams.maxPropSpeed); 
    checkResult(result.status, 'omnimorph.set_wlimit', result.exception);

    result = omnimorph.set_control_mode('::omnimorph::full_attitude');
    checkResult(result.status, 'omnimorph.set_control_mode', result.exception);

    if controllerChoice == 4
        result = omnimorph.set_pose_control_mode('::omnimorph::miqp');
        checkResult(result.status, 'omnimorph.set_pose_control_mode MIQP', result.exception);
    end
    
    result = omnimorph.set_servo_gain(config.conrtolParams.gainsPID.Kp(1,1), ...
                                      config.conrtolParams.gainsPID.Kp(3,3), ...
                                      config.conrtolParams.gainsPID.KR(1,1), ...
                                      config.conrtolParams.gainsPID.KR(3,3), ...
                                      config.conrtolParams.gainsPID.Kv(1,1), ...
                                      config.conrtolParams.gainsPID.Kv(3,3), ...
                                      config.conrtolParams.gainsPID.Kw(1,1), ...
                                      config.conrtolParams.gainsPID.Kw(3,3), ...
                                      config.conrtolParams.gainsPID.KI_p(1,1), ...
                                      config.conrtolParams.gainsPID.KI_p(3,3));
    checkResult(result.status, 'omnimorph.set_servo_gain', result.exception);
    
    result = omnimorph.set_saturation(config.conrtolParams.errSat.e_p, ...
                                      config.conrtolParams.errSat.e_v, ...
                                      config.conrtolParams.errSat.e_i);
    checkResult(result.status, 'omnimorph.set_saturation', result.exception);

    resul = omnimorph.set_emerg(config.conrtolParams.emerg.descent, ...
                                config.conrtolParams.emerg.dx, ...
                                config.conrtolParams.emerg.dq, ...
                                config.conrtolParams.emerg.dv, ...
                                config.conrtolParams.emerg.dw); 
    checkResult(result.status, 'omnimorph.set_emerg', result.exception);
end

%% Maneuver
if experimentFlag
    maneuver = client.load(strcat(remoteDevelPath,'maneuver'));
else
    maneuver = client.load(strcat(develPath,'maneuver'));
end
pause(0.5)

if controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4
    result = maneuver.connect_port('state', 'pom/frame/robot');
    checkResult(result.status, 'maneuver.connect_port to POM', result.exception);

    if controllerChoice == 3
        result = omnimorph.connect_port('reference','maneuver/desired');
        checkResult(result.status, 'omnimorph.connect_port to maneuver desired', result.exception);
    end

    result = maneuver.set_bounds(config.trajParams.trajXMin, ...
                                 config.trajParams.trajXMax, ...
                                 config.trajParams.trajYMin, ...
                                 config.trajParams.trajYMax, ...
                                 config.trajParams.trajZMin, ...
                                 config.trajParams.trajZMax, ...
                                 config.trajParams.trajYawMin, ...
                                 config.trajParams.trajYawMax);
    checkResult(result.status, 'maneuver.set_bounds', result.exception);

    result = maneuver.set_velocity_limit(config.trajParams.trajVMax, ...
                                         config.trajParams.trajWMax);
    checkResult(result.status, 'maneuver.set_velocity_limit', result.exception);

    result = maneuver.set_acceleration_limit(config.trajParams.trajAMax, ...
                                             config.trajParams.trajDwMax);
    checkResult(result.status, 'maneuver.set_acceleration_limit', result.exception);
end

%% Physical interaction setup
if experimentFlag
    phynt = client.load(strcat(remoteDevelPath,'phynt'));
else
    phynt = client.load(strcat(develPath,'phynt'));
end
pause(0.5)

if controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4
    result = phynt.connect_port('state','pom/frame/robot');
    checkResult(result.status, 'phynt.connect_port to POM', result.exception);

    result = phynt.connect_port('wrench_measure', 'omnimorph/wrench_measure');
    checkResult(result.status, 'phynt.connect_port to omnimorph wrench measure', result.exception);

    result = phynt.connect_port('reference','maneuver/desired');
    checkResult(result.status, 'phynt.connect_port reference to maneuver', result.exception);

    if controllerChoice == 2 || controllerChoice == 4
        result = omnimorph.connect_port('reference', 'phynt/desired');
        checkResult(result.status, 'omnimorph.connect_port to phynt desired', result.exception);
    end
    
    result = omnimorph.connect_port('external_wrench','phynt/external_wrench');
    checkResult(result.status, 'connect phynt external wrench port to omni', result.exception);    

    result = phynt.set_mass(config.uavParams.mass);
    checkResult(result.status, 'phynt.set_mass', result.exception);

    result = phynt.set_geom(num2cell(reshape(config.uavParams.inertia',[9,1])));
    checkResult(result.status, 'phynt.set_geom', result.exception);

    result = phynt.set_ee(num2cell(config.uavParams.endEffector.p_e), ...
                          num2cell(reshape(config.uavParams.endEffector.R_e',[9,1])));
    checkResult(result.status, 'phynt.set_ee', result.exception);

    result = phynt.set_wo_gains(num2cell(diag(config.conrtolParams.wrenchObs.gains)));
    checkResult(result.status, 'phynt.set_wo_gains', result.exception);

    result = phynt.set_af_parameters(config.conrtolParams.admitFilt.virtMass(1,1), ...
                                     num2cell(diag(config.conrtolParams.admitFilt.virtDamp)), ...
                                     num2cell(diag(config.conrtolParams.admitFilt.virtStiff)), ...
                                     num2cell(reshape(config.conrtolParams.admitFilt.virtMass(4:6,4:6),[9,1])));
    checkResult(result.status, 'phynt.set_af_parameters', result.exception);

    result = phynt.set_wo_fc(num2cell(config.conrtolParams.wrenchObs.cutOff));
    checkResult(result.status, 'phynt.set_wo_fc', result.exception);
    
    result = phynt.set_wo_thresh(num2cell(config.conrtolParams.wrenchObs.thres));
    checkResult(result.status, 'phynt.set_wo_thres', result.exception);    

    result = phynt.set_wo_bias(num2cell(config.conrtolParams.wrenchObs.bias));
    checkResult(result.status, 'phynt.set_wo_bias', result.exception);
end

%% Battery check and Pom output
batteryLevel = rotorcraft.get_battery();
fprintf(strcat('Battery level: ', num2str(batteryLevel.result.battery.level),'\n'));

fprintf('Pom position: \n');
fprintf(strcat('x: ',num2str(pom.frame('robot').frame.pos.x),'\n'));
fprintf(strcat('y: ',num2str(pom.frame('robot').frame.pos.y),'\n'));
fprintf(strcat('z: ',num2str(pom.frame('robot').frame.pos.z),'\n'));
fprintf('\n');

fprintf('Pom velocity: \n');
fprintf(strcat('vx: ',num2str(pom.frame('robot').frame.vel.vx),'\n'));
fprintf(strcat('vy: ',num2str(pom.frame('robot').frame.vel.vy),'\n'));
fprintf(strcat('vz: ',num2str(pom.frame('robot').frame.vel.vz),'\n'));
fprintf('\n');

fprintf('Pom acceleration: \n');
fprintf(strcat('ax: ',num2str(pom.frame('robot').frame.acc.ax),'\n'));
fprintf(strcat('ay: ',num2str(pom.frame('robot').frame.acc.ay),'\n'));
fprintf(strcat('az: ',num2str(pom.frame('robot').frame.acc.az),'\n'));
fprintf('\n');

rpy = quat2eul(cell2mat(struct2cell(pom.frame('robot').frame.att))');
fprintf('Pom attitude: \n');
fprintf(strcat('roll: ',num2str(rad2deg(rpy(3))),'\n'));
fprintf(strcat('pitch: ',num2str(rad2deg(rpy(2))),'\n'));
fprintf(strcat('yaw: ',num2str(rad2deg(rpy(1))),'\n'));
fprintf('\n');

fprintf('Pom angular velocity: \n');
fprintf(strcat('wx: ',num2str(pom.frame('robot').frame.avel.wx),'\n'));
fprintf(strcat('wy: ',num2str(pom.frame('robot').frame.avel.wy),'\n'));
fprintf(strcat('wz: ',num2str(pom.frame('robot').frame.avel.wz),'\n'));
fprintf('\n');

if controllerChoice == 1 
    rotorcraft.start();
end 

run start_sequence_com.m

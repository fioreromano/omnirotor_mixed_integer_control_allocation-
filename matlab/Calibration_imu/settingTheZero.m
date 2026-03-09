%% Config 
%list of possible hosts 
    host_name = '192.168.0.110'; %FiberTHex plus
%   host_name = '192.168.0.181'; %FiberTHex 
% Uncomment this and comment the previous one, if
% everything in running on your pc.
% host_name = 'localhost' 
openrobots_dir = '$ROBOTPKG_BASE/openrobots'; % Path to openrobots directory in the robot's pc
resultDir = 'results'; % Specify directory in which calibration result has to be saved

%% Create path to saving-result directory
% If resultDir is not in pwd then change this section accordingly!

if strcmp(resultDir, '') ~= 1 % If any folder has been specified
    % Path to saving-results folder
    path2ResultDir = strcat(pwd, filesep, resultDir); % append filesep and that folder to path 
else
    path2ResultDir = pwd;
end

%% Setup client

disp('* Retrieve client...');
client = genomix.client(host_name); 
client.rpath([openrobots_dir, '/lib/genom/pocolibs/plugins']);
disp('done');

%% Setup rotorcraft

disp('* Initialize rotorcraft...');
rc = client.load('rotorcraft');
pause(1);
disp('done');
msg_conn_rc = rc.connect('/dev/ttyUSB0', 500000);
string = ['* Connect to rotorcraft: ',msg_conn_rc.status];
disp(string);


%% Load unzereod calib file
load(strcat('fbthexplus_calib.mat'));
    calibration = eval('calibration');
    % modify astdev
    for i=1:1:3
       calibration.imu_calibration.astddev{i} = ...
        10*calibration.imu_calibration.astddev{i};        
    end
    calibration.imu_calibration.mscale= {0,0,0,0,0,0,0,0,0};
    cal_res = rc.set_imu_calibration(calibration);
pause(1);

 %% Set to zero the accelerometers calibration values
    disp('* Zero the accelerometer values');
    disp(' Put the aerial platform on the ground or on another flat surface parallel, and align its X arm (red one) ');
    disp('  with X axis of World frame.');
    disp('> X axis in World frame is pointing towards the windows looking at the outdoor arena!');
    disp('> Use the markers under the foam in arena to align the aerial platform correctly!');
    disp('  to it!');
    input(' Once you have done it, press ENTER ', 's');
    rc.set_zero();
    disp('done');
    
 %% Get calibration of IMU and re-save it
    disp('* Get IMU calibration...');
    calibration = rc.get_imu_calibration();
    calibration = calibration.result;
    disp('done');
    
    % Save calibration result
    disp('* Save result...');
    if strcmp(path2ResultDir, '') ~= 1 % If saving directory has been specified
        if exist(path2ResultDir, 'dir') ~= 7 % if saving directory does not exist
            mkdir(path2ResultDir); % create it!
        end
        path2ResultDir = strcat(path2ResultDir, filesep); % append filesep
    end
    filename = input(' Specify here the filename for saving the result: ', 's');
    filename = strcat(path2ResultDir, filename);
    save(filename, 'calibration');
    fprintf(' Calibration result saved in:\n  %s\n', path2ResultDir);
    disp('done');   
    
 %% Test result   
 
 % Test calibration result
    disp(' ');
    test = input('Would like to test the result? (y/n): ', 's');
    
    disp('* Test');
    if strcmpi(test, 'y')
    
        do = true;
        while do
            
            disp(' Accelerometer:');
            imu = rc.imu();
            imu.imu.acc
            disp('-----');
            
            skip = input(' Press enter to get another sample or type < n > for exit: ' , 's');
            
            if strcmpi(skip, 'n') == 1
                do = false;
            end
        end 
        
    end
    disp('done');
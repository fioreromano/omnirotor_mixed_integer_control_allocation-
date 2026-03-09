clear all;
close all;
clc;

%% Config 
%list of possible hosts 
%   host_name = '192.168.0.110'; %FiberTHex plus
%   host_name = '192.168.0.181'; %FiberTHex 
% Uncomment this and comment the previous one, if
% everything in running on your pc.
%host_name = 'localhost' ;
host_name = '192.168.0.122';    % omnimorph
%openrobots_dir = '$ROBOTPKG_BASE/openrobots'; % Path to openrobots directory in the robot's pc
openrobots_dir = '/home/fiore/openrobots';
resultDir = 'results'; % Specify directory in which calibration result has to be saved

%% local machine paths
username = 'fiore';
% Put the custom path that you used for the ROBOTPKG_BASE on your localhost
ROBOTPKG_BASE = strcat('/home/', username, '/openrobots/');
DEVEL_BASE = strcat('/home/', username, '/devel/');

or_path = strcat(ROBOTPKG_BASE, 'lib/genom/pocolibs/plugins/'); 
devel_path = strcat(DEVEL_BASE, 'lib/genom/pocolibs/plugins/');

%% Create path to saving-result directory
% If resultDir is not in pwd then change this section accordingly!

if strcmp(resultDir, '') ~= 1 % If any folder has been specified
    % Path to saving-results folder
    path2ResultDir = strcat(pwd, filesep, resultDir); % append filesep and that folder to path 
else
    path2ResultDir = pwd;
end

%% Add genomix paths
addpath(strcat(ROBOTPKG_BASE,'lib/matlab/simulink/genomix/'))
addpath(strcat(ROBOTPKG_BASE,'lib/matlab/simulink/'))
addpath(strcat(ROBOTPKG_BASE,'lib/matlab/'))

%% Setup client

disp('* Retrieve client...');
client = genomix.client(host_name); 
client.rpath([openrobots_dir, '/lib/genom/pocolibs/plugins']);
disp('done');

%% Setup rotorcraft

disp('* Initialize rotorcraft...');
rc = client.load('rotorcraft');
%rc = client.load(strcat(or_path,'rotorcraft'));
%usb_port = 'chimera-18'; %fiberThex
usb_port = 'chimera-115'; %miniThex
pause(1);
disp('done');
msg_conn_rc = rc.connect(usb_port, 0);

string = ['* Connect to rotorcraft: ',msg_conn_rc.status];
disp(string);

%% IMU CALIBRATION
disp('* Calibrate imu...');

calibrating = true;
abort = false;

while calibrating
    disp(' ');
    disp(' Before starting: remember to keep visible the terminal in which');
    disp('  you have launched the sh script!');
    disp('Follow the instruction on that terminal, once calibration has started!.');
    disp(' ');
    temp = input(' Press ENTER when you are ready to calibrate the IMU. ', 's');
    rc.calibrate_imu(2, 10);
    

    success = input(' Have you succeed in calibrating the IMU? (y/n): ', 's');
    
    if strcmpi(success, 'y') == 1
        calibrating = false;
    else
        retry = input(' Would you like to retry? (y/n): ', 's');   
        if strcmpi(retry, 'n') == 1
            abort = true;
            calibrating = false;
        end 
    end
end

if ~abort
    
    % Calibration of IMU complete
    disp('done'); 
    
    % Set to zero the accelerometers calibration values
    disp('* Zero the accelerometer values');
    disp(' Put the aerial platform on the ground or on another flat surface parallel, and align its X arm (red one) ');
    disp('  with X axis of World frame.');
    disp('> X axis in World frame is pointing towards the windows looking at the outdoor arena!');
    disp('> Use the markers under the foam in arena to align the aerial platform correctly!');
    disp('  to it!');
    input(' Once you have done it, press ENTER ', 's');
    rc.set_zero();
    disp('done');
    
    % Get calibration of IMU
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
    
    %%
    % Load calibration result
     disp('* Load calibration result...');
     load(filename);
     calibration = eval('calibration');
     disp('done')

     % correct for empty temp setting
     calibration.imu_calibration.temp = 0;
    
    % Correct values
    disp('* Correct standard deviation values on accelerometer...');
    for i = 1 : 1 : 3
        calibration.imu_calibration.astddev{i} = ...
            10 * calibration.imu_calibration.astddev{i}; 
    end
    disp('done');


    
    % Apply calibration result
    disp('* Apply calibration result...');
    calibration.imu_calibration.mscale= {0,0,0,0,0,0,0,0,0};
    rc.set_imu_calibration(calibration);
    disp('done');

    %%
    
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
else
    
    disp('Aborted!');

end

disp('* End');
%% Initial offset
R_in = quat2rotm(cell2mat(struct2cell(pom.frame('robot').frame.att))');
eul_in = quat2eul(cell2mat(struct2cell(pom.frame('robot').frame.att))');

if controllerChoice == 2 || controllerChoice == 4
    posOffset = [pom.frame('robot').frame.pos.x;
                 pom.frame('robot').frame.pos.y;
                 pom.frame('robot').frame.pos.z] + R_in*config.uavParams.endEffector.p_e;
    bodPosOffset = [pom.frame('robot').frame.pos.x;
                    pom.frame('robot').frame.pos.y;
                    pom.frame('robot').frame.pos.z];
elseif controllerChoice == 3
    posOffset = [pom.frame('robot').frame.pos.x;
                 pom.frame('robot').frame.pos.y;
                 pom.frame('robot').frame.pos.z];
    bodPosOffset = posOffset;
end

%% Generate trajectory
liftOff = [0;0;0.5];

trajChoice = lower(input('Generate trajectory? (y/n): ', 's'));

% Validate trajectory generation input
while ~ismember(trajChoice, {'y', 'n'})
    fprintf('Invalid input. Please enter "yes" or "no".');
    trajChoice = lower(input('Generate trajectory? (y/n): ', 's'));
end

trajFlag = strcmp(trajChoice, 'y');
if trajFlag
    trajName = input('Enter a name for the trajectory: ', 's');
    trajName = strcat(trajName,'.log');

    if exist(strcat(trajPath,trajName),'file')
        warning('The file "%s" already exists. Log will be overwritten.', trajName);
        proceed = lower(input('Do you want to continue? (y/n): ', 's'));
        if ~strcmp(proceed, 'y')
            error('Trajectory generation aborted by user.');
        end
    end

    config.trajParams.wayPoints = config.trajParams.wayPoints - R_in*config.uavParams.CoM;
    config.trajParams.wayPoints(:,1) = posOffset+liftOff - R_in*config.uavParams.CoM;
    config.trajParams.wayPoints(:,end) = posOffset+liftOff - R_in*config.uavParams.CoM;
    [pdes, vdes, ades, tdes] = minJerkPloyTraj(config.trajParams.wayPoints, ...
                                               config.trajParams.initVel, ...
                                               config.trajParams.initAcc, ...
                                               config.trajParams.timePoints, ...
                                               config.trajParams.numSamples);

    config.trajParams.rotPoints(:,:,1) = eye(3);
    config.trajParams.rotPoints(:,:,end) = eye(3);
    [Rdes, omd, domd, tdes] = minAaccPolyTraj(config.trajParams.rotPoints, ...
                                        config.trajParams.initOm, ...
                                        config.trajParams.timePoints, ...
                                        config.trajParams.numSamples);

    trajToLog(pdes, vdes, ades, Rdes, omd, tdes, strcat(trajPath,trajName));

else
    trajName = input('Enter a name of the trajectory to use: ', 's');
    trajName = strcat(trajName,'.log');

    while ~exist(strcat(trajPath,trajName),'file')
        fprintf('Invalid input. Enter existing trajecotry. \n');
        trajName = input('Enter a name of the trajectory to use: ', 's');
        trajName = strcat(trajName,'.log');
    end
end
fprintf('\n');

% Copy trajectory locally to data folder
if logFlag
    status = system(sprintf('cp %s %s', strcat(trajPath,trajName), baseFolder));
    if status == 0
        fprintf('Trajectory file locally copied successfully.\n');
    else
        fprintf('Trajectory file local copy failed.\n');     
    end
end

% Copying trajecotry to Raspi
if experimentFlag 
    fprintf('Copying trajectory to Raspi... \n');
    status = system(sprintf('scp %s %s@%s:%s', strcat(trajPath,trajName), raspiUser, remoteIP, remoteTrajPath));
    if status == 0
        fprintf('Trajectory file copy to Raspi successfull.\n');
    else
        fprintf('Trajectory file copy to Raspi failed.\n');
        fprintf('To copy trajectory to Raspi from local machine manually: \n')
        fprintf('scp %s %s@%s:%s \n', strcat(trajPath,trajName), raspiUser, remoteIP, remoteTrajPath); 
    end
end

%% Start 
reply = lower(input('WARNING: rotorcraft is about to start, make sure to launch and test the emergency stop script. Are you ready? (y/n): ', 's'));
if ~strcmp(reply, 'y')
    error('Starting aborted by user.');
end

% Start logging
if logFlag && (controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4)
    maneuver.log(strcat(logPath,'man.log'));
    phynt.log(strcat(logPath,'phyn.log'));
    pom.log_state(strcat(logPath,'stats.log'));
    rotorcraft.log(strcat(logPath,'rot.log'));
    omnimorph.log(strcat(logPath,'omni.log'));
    optitrack.set_logfile(strcat(logPath,'opt.log'));
end

% Start motors
rotorcraft.start();
pause(0.1)

if controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4
    % Start servoing reference (send commands asynchronously)
    result = omnimorph.servo('-a');
    checkResult(result.status, 'omnimorph servo', result.exception);
    pause(0.1)
    
    result = phynt.servo('-a');
    checkResult(result.status, 'phynt servo', result.exception);
    pause(0.1)

    result = rotorcraft.servo('-a');
    checkResult(result.status, 'rotorcraft servo', result.exception);
    pause(0.1)

    % Set current state as desired
    result = maneuver.set_state(posOffset(1),posOffset(2),posOffset(3),eul_in(1));
    checkResult(result.status, 'maneuver set state', result.exception);
    pause(0.1)

    % Disable wrench observer to prevent spike in force feeding into admittance filter
    result = phynt.enable(0,1);
    checkResult(result.status, 'phynt.enable', result.exception);

    % Start tracking 
    result = maneuver.goto(posOffset(1),posOffset(2),posOffset(3),eul_in(1),5);
    checkResult(result.status, 'maneuver lift-off', result.exception);

    % Lift-off
    result = maneuver.goto(posOffset(1),posOffset(2),posOffset(3)+liftOff(3),eul_in(1),0);
    checkResult(result.status, 'maneuver lift-off', result.exception);
    pause(0.2)

    % Enable wrench observer
    result = phynt.enable(1,1);
    checkResult(result.status, 'phynt.enable', result.exception);

    % Replay trajectory
    if experimentFlag
        result = maneuver.replay(strcat(remoteTrajPath,trajName));
        checkResult(result.status, 'maneuver replay', result.exception);
    else
        result = maneuver.replay(strcat(trajPath,trajName));
        checkResult(result.status, 'maneuver replay', result.exception);
    end

    % Land
    result = maneuver.goto(posOffset(1),posOffset(2),posOffset(3),eul_in(1),0);
    checkResult(result.status, 'maneuver land', result.exception);
    pause(0.2)
end

%% Stop all components
if controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4
    result = rotorcraft.stop('-a');
    checkResult(result.status, 'rotorcraft stop', result.exception);

    result = rotorcraft.stop('-a');
    checkResult(result.status, 'rotorcraft stop', result.exception);

    result = maneuver.stop('-a');
    checkResult(result.status, 'maneuver stop', result.exception);

    result = phynt.stop('-a');
    checkResult(result.status, 'phynt stop', result.exception);

    result = omnimorph.stop('-a');
    checkResult(result.status, 'omnimorph stop', result.exception);

    result = omnimorph.stop('-a');
    checkResult(result.status, 'omnimorph stop', result.exception);    
end

if logFlag && (controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4)
    maneuver.log_stop();    
    phynt.log_stop();
    pom.log_stop();
    rotorcraft.log_stop();
    omnimorph.log_stop();
    optitrack.unset_logfile();
end

%% Copying logs and clean up
if experimentFlag && (logFlag && (controllerChoice == 2 || controllerChoice == 3 || controllerChoice == 4))
    fprintf('Copying logs... \n');
    status = system(sprintf('scp %s@%s:%s* %s', raspiUser, remoteIP, logPath, logFolder));
    if status == 0
        fprintf('Log files copied successfully.\n');
    else
        fprintf('Failed to copy log files.\n');
        fprintf('To copy log files from Raspi to local machine manually: \n')
        fprintf('scp %s@%s:%s* %s \n', raspiUser, remoteIP, logPath, logFolder);        
    end
    
    status = system(sprintf('scp %s@%s:%s %s', raspiUser, remoteIP, pocolibsPath, logFolder));
    if status == 0
        fprintf('Pocolibs log copied successfully.\n');
    else
        fprintf('Failed to copy Pocolibs log.\n');
        fprintf('To copy pocolibs log from Raspi to local machine manually: \n')
        fprintf('scp %s@%s:%s %s \n', raspiUser, remoteIP, pocolibsPath, logFolder);        
    end

    fprintf('Cleaning up... \n');
    status = system(sprintf('ssh %s@%s "rm %s*.log"', raspiUser, remoteIP, logPath));
    if status == 0
        fprintf('Genom3 logs deleted successfully.\n');
    else
        fprintf('Failed to delete logs.\n');
        fprintf('To delete logs from Raspi manually: \n')
        fprintf('ssh %s@%s "rm %s*.log" \n', raspiUser, remoteIP, logPath);        
    end
end

if experimentFlag
    status = system(sprintf('ssh %s@%s "rm %s*.log"', raspiUser, remoteIP, remoteTrajPath));
    if status == 0
        fprintf('Trajectory file deleted successfully.\n');
    else
        fprintf('Failed to delete trajectory.\n');
        fprintf('To delete trajectory from Raspi manually: \n')
        fprintf('ssh %s@%s "rm %s*.log" \n', raspiUser, remoteIP, remoteTrajPath);        
    end
end
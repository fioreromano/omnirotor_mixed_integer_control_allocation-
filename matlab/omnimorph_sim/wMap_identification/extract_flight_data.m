%% Check if baseFolder exists, otherwise ask user to select a folder
if ~exist('baseFolder', 'var') || isempty(baseFolder)
    baseFolder = uigetdir(pwd, 'Select folder containing data files');
    if isequal(baseFolder, 0)
        error('No folder selected. Aborting log extraction.');
    end
    % Ensure trailing slash
    if baseFolder(end) ~= filesep
        baseFolder = [baseFolder filesep];
    end

    logFolder = strcat(baseFolder,'logs/');
    figFolder = strcat(baseFolder,'figures/');
    datFolder = strcat(baseFolder,'data/');

    addpath('../lib/')
    addpath('../lib/omnimorph-helper/')
    addpath('../lib/math-helper/')
    addpath('../sim/')
    addpath('../plot/')

    run('../omnimorph_params')
end

%% Extract data
extractLog(strcat(logFolder, 'rot.log'),'rotors');
extractLog(strcat(logFolder, 'omni.log'),'omni');
extractLog(strcat(logFolder, 'stats.log'),'pom');

% Rotorcraft data processing
valid = ~isnan(rotors.ts) & ~isnan(rotors.meas_v0) & ~isnan(rotors.meas_v1) & ~isnan(rotors.meas_v2) & ~isnan(rotors.meas_v3) & ~isnan(rotors.meas_v4) & ~isnan(rotors.meas_v5) & ~isnan(rotors.meas_v6) & ~isnan(rotors.meas_v7);

% Extract times
globalStart = min([pom.ts(1), rotors.ts(1), omni.ts(1)]);
pomTime   = pom.ts   - globalStart;
rotTime = rotors.ts(valid) - globalStart;
omniTime = omni.ts - globalStart;

maxPropeller = ones(1,numel(rotTime))*config.uavParams.maxPropSpeed;
minPropeller = ones(1,numel(rotTime))*config.uavParams.minPropSpeed;

% Filtering rotor speeds
% Design Butterworth filter
fs = 1/mean(diff(rotTime));  % sampling frequency (Hz)
fc = 5;                     % cutoff frequency (Hz), tune as needed
[b,a] = butter(4, fc/(fs/2)); % 4th order low-pass filter

% Apply zero-phase filtering
rotors.meas_v0 = filtfilt(b, a, rotors.meas_v0(valid));
rotors.meas_v1 = filtfilt(b, a, rotors.meas_v1(valid));
rotors.meas_v2 = filtfilt(b, a, rotors.meas_v2(valid));
rotors.meas_v3 = filtfilt(b, a, rotors.meas_v3(valid));
rotors.meas_v4 = filtfilt(b, a, rotors.meas_v4(valid));
rotors.meas_v5 = filtfilt(b, a, rotors.meas_v5(valid));
rotors.meas_v6 = filtfilt(b, a, rotors.meas_v6(valid));
rotors.meas_v7 = filtfilt(b, a, rotors.meas_v7(valid));

% Plot all data
% Visualization parameters
lineSize = 2;
legFontSize = 20;

fig = figure('name','Force torque data', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
ax1 = subplot(3,1,1);
hold on
grid on
plot(rotTime,rotors.meas_v0,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotTime,rotors.meas_v1,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotTime,rotors.meas_v2,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotTime,rotors.meas_v3,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotTime,rotors.meas_v4,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotTime,rotors.meas_v5,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotTime,rotors.meas_v6,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotTime,rotors.meas_v7,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));
    
plot(rotTime,maxPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','upper limit','$$'))
plot(rotTime,minPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','lower limit','$$'))
    
leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-10);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('propeller speed [Hz] ','FontSize',legFontSize)
title('Measured Propeller speeds Vs time','FontSize',legFontSize)      

ax2 = subplot(3,1,2);
hold on
grid on
plot(pomTime,pom.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{B}','$$'));
plot(pomTime,pom.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{B}','$$'));
plot(pomTime,pom.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{B}','$$'));

plot(omniTime,omni.xd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{B}','$$'));
plot(omniTime,omni.yd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{B}','$$'));
plot(omniTime,omni.zd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{B}','$$'));

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('pos [meters]','FontSize',legFontSize)
title('UAV body (CoM) position Vs time','FontSize',legFontSize)

ax3 = subplot(3,1,3);
hold on
grid on
plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));

plot(omniTime,rad2deg(unwrap(omni.rolld)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{B}','$$'));
plot(omniTime,rad2deg(unwrap(omni.pitchd)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{B}','$$'));
plot(omniTime,rad2deg(unwrap(omni.yawd)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{B}','$$'));

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize)
title('UAV body orientation vs time','FontSize',legFontSize)
    
linkaxes([ax1, ax2, ax3], 'x');

datNum = 1;
while true
    % Let user click two points on the x-axis
    disp('Select two points to define time range');
    [xRange, ~] = ginput(2);
        
    xMin = min(xRange);
    xMax = max(xRange);
        
    % Add vertical lines to show selection
    xline(xMin, 'r--', 'LineWidth', lineSize, 'HandleVisibility', 'off');
    xline(xMax, 'r--', 'LineWidth', lineSize, 'HandleVisibility', 'off');

    % Extract rotorcraft data in range
    idx = rotTime >= xMin & rotTime <= xMax;

    % Compute input between points
    in = [rotors.meas_v0';
          rotors.meas_v1';
          rotors.meas_v2';
          rotors.meas_v3';
          rotors.meas_v4';
          rotors.meas_v5';
          rotors.meas_v6';
          rotors.meas_v7';];
    in = mean(in(:,idx),2);
    measIn = sign(in) .* in.^2;

    % Extract rotorcraft data in range
    idx = pomTime >= xMin & pomTime <= xMax;    

    % Extract roll, pitch, yaw
    yaw = mean(pom.yaw(idx));  % [Z Y X]
    pitch = mean(pom.pitch(idx));
    roll = mean(pom.roll(idx));

    % Compute rotation matrix
    R = eul2rotm([yaw,pitch,roll], 'ZYX');    

    % Body thrust
    bodForce = R' * [0;0;config.uavParams.mass*config.gravity];

    measWrench = [bodForce;0;0;0];

    % Save datapoint in struct
    datName = strcat('dat_',num2str(datNum));
    dataStruct = struct();
    dataStruct.(datName).measIn = measIn;
    dataStruct.(datName).measWrench = measWrench;
    save(strcat(datFolder,datName,'.mat'),'dataStruct');    

    % Ask if the user wants to continue
    answer = questdlg('Do you want to select more points?', ...
                      'Continue?', ...
                      'Yes', 'No', 'No');
        
    if strcmp(answer, 'No')
        break;
    end

    datNum=datNum+1;
end

saveas(fig,strcat(figFolder,'dat.png'));    

%% Plot measured vs commanded thrust vectors

load(strcat(baseFolder,"Rots.mat"))

figure(2)
hold on
view(3)
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
origin = [0,0,0];

for i = 1:numel(Rots(1,1,:))
    v = Rots(:,:,i)' * [0;0;config.uavParams.mass*config.gravity];

    quiver3(origin(1), origin(2), origin(3), ...
            v(1),      v(2),      v(3), ...
            0, 'LineWidth', 2, 'Color', "blue");
    drawnow
    pause(0.1)

    datName = strcat('dat_',num2str(i));

    try
        load(strcat(datFolder,datName,'.mat'))
        v = dataStruct.(datName).measWrench(1:3);
    
        quiver3(origin(1), origin(2), origin(3), ...
                v(1),      v(2),      v(3), ...
                0, 'LineWidth', 2, 'Color', "red");
        drawnow
        pause(0.1)
    catch
        warning('No corresponding data')
    end
    
end
legend('Intended thrust', 'Measured thrust');

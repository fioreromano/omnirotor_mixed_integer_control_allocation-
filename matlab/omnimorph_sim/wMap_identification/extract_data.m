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
fileList = dir(fullfile(logFolder, '*.log'));
numLogs =  numel(fileList);

while true
    choice = input('Type c to continue extracting data, or q to quit: ', 's');

    if strcmpi(choice, 'q')
        disp('Exiting…');
        break
    end

    logChoiceStr = input(sprintf('Choose log to extract (1-%s): ', num2str(numLogs)), 's');

    logChoice = str2double(logChoiceStr);
    if isnan(logChoice) || logChoice < 1 || logChoice > numLogs || logChoice ~= floor(logChoice)
        fprintf('"%s" is not a valid integer between 1 and %s.\n', num2str(logChoice), num2str(numLogs));
        continue          % restart loop and ask again
    end    

    % Extract genom3 and FT data
    extractLog(strcat(logFolder, 'rot_',logChoiceStr,'.log'),'rotors')
    extractFTSensorData(strcat(logFolder, 'FT_',logChoiceStr),'FT')

    % Rotorcraft data processing
    valid = ~isnan(rotors.ts) & ~isnan(rotors.meas_v0) & ~isnan(rotors.meas_v1) & ~isnan(rotors.meas_v2) & ~isnan(rotors.meas_v3) & ~isnan(rotors.meas_v4) & ~isnan(rotors.meas_v5) & ~isnan(rotors.meas_v6) & ~isnan(rotors.meas_v7);
    rotDatTime = datetime(rotors.ts(valid),'ConvertFrom','posixtime','TimeZone','Europe/Amsterdam','Format','DDD:HH:mm:ss.SSS');

    maxPropeller = ones(1,numel(rotDatTime))*config.uavParams.maxPropSpeed;
    minPropeller = ones(1,numel(rotDatTime))*config.uavParams.minPropSpeed;

    % Compute wrench according to nominal wrench map for comparison
    expectedWrench = config.uavParams.wrenchMap*[(sign(rotors.meas_v0(valid)).*((rotors.meas_v0(valid)).^2))';
                                                 (sign(rotors.meas_v1(valid)).*((rotors.meas_v1(valid)).^2))';
                                                 (sign(rotors.meas_v2(valid)).*((rotors.meas_v2(valid)).^2))';
                                                 (sign(rotors.meas_v3(valid)).*((rotors.meas_v3(valid)).^2))';
                                                 (sign(rotors.meas_v4(valid)).*((rotors.meas_v4(valid)).^2))';
                                                 (sign(rotors.meas_v5(valid)).*((rotors.meas_v5(valid)).^2))';
                                                 (sign(rotors.meas_v6(valid)).*((rotors.meas_v6(valid)).^2))';
                                                 (sign(rotors.meas_v7(valid)).*((rotors.meas_v7(valid)).^2))'];    

    % FT sensor data processing
    % Wrench at the sensor frame such that it coincides with the body frame
    sensWrench = [FT.Fy; -FT.Fx; FT.Fz; FT.Ty; -FT.Tx; FT.Tz];   

    % Transform wrench to body frame 
    p = [0;0;-0.047]; % vector to body frame in sensor frame in sensor frame
    R = eye(3); % Rot matrix from sensor to body
    Ad = [R,            zeros(3);
          skewMat(R*p), R        ];
    bodWrench = Ad*sensWrench;
    
    % Align genom3 and FT times
    globalStart = min([rotDatTime(1),FT.ts(1)]);
    rotSecTime = seconds(rotDatTime - globalStart);
    FTSecTime = seconds(FT.ts - globalStart);    
    
    % Plot all data
    % Visualization parameters
    lineSize = 2;
    legFontSize = 20;

    fig = figure('name','Force torque data', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
    set(fig,'defaulttextinterpreter','latex');
    ax1 = subplot(3,1,1);
    hold on
    grid on
    plot(rotSecTime,rotors.meas_v0(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
    plot(rotSecTime,rotors.meas_v1(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
    plot(rotSecTime,rotors.meas_v2(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
    plot(rotSecTime,rotors.meas_v3(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
    plot(rotSecTime,rotors.meas_v4(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
    plot(rotSecTime,rotors.meas_v5(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
    plot(rotSecTime,rotors.meas_v6(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
    plot(rotSecTime,rotors.meas_v7(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));
    
    plot(rotSecTime,maxPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','upper limit','$$'))
    plot(rotSecTime,minPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','lower limit','$$'))
    
    leg = legend('show','Location','NorthEast');
    set(leg,'Interpreter','latex','FontSize',legFontSize-10);
    set(gca,'FontSize',legFontSize-10);
    xlabel('time [sec]','FontSize',legFontSize)
    ylabel('propeller speed [Hz] ','FontSize',legFontSize)
    title('Measured Propeller speeds Vs time','FontSize',legFontSize)      
    
    ax2 = subplot(3,1,2);
    hold on
    grid on
    plot(FTSecTime,bodWrench(1,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{x}^{sens}','$$'));
    plot(FTSecTime,bodWrench(2,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{y}^{sens}','$$'));
    plot(FTSecTime,bodWrench(3,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{z}^{sens}','$$'));
    plot(rotSecTime,expectedWrench(1,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{x}^{mot}','$$'));
    plot(rotSecTime,expectedWrench(2,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{y}^{mot}','$$'));
    plot(rotSecTime,expectedWrench(3,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{z}^{mot}','$$'));
    
    leg = legend('show','Location','NorthEast');
    set(leg,'Interpreter','latex','FontSize',legFontSize-10);
    set(gca,'FontSize',legFontSize-10);
    xlabel('time [sec]','FontSize',legFontSize)
    ylabel('Force [N] ','FontSize',legFontSize)
    title('Measured Body force Vs time','FontSize',legFontSize)    
    
    ax3 = subplot(3,1,3);
    hold on
    grid on
    plot(FTSecTime,bodWrench(4,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','m_{x}^{sens}','$$'));
    plot(FTSecTime,bodWrench(5,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','m_{y}^{sens}','$$'));
    plot(FTSecTime,bodWrench(6,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','m_{z}^{sens}','$$'));
    plot(rotSecTime,expectedWrench(4,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','m_{x}^{mot}','$$'));
    plot(rotSecTime,expectedWrench(5,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','m_{y}^{mot}','$$'));
    plot(rotSecTime,expectedWrench(6,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','m_{z}^{mot}','$$')); 
    
    leg = legend('show','Location','NorthEast');
    set(leg,'Interpreter','latex','FontSize',legFontSize-10);
    set(gca,'FontSize',legFontSize-10);
    xlabel('time [sec]','FontSize',legFontSize)
    ylabel('Torque [Nm] ','FontSize',legFontSize)
    title('Measured Body torque Vs time','FontSize',legFontSize)   
    
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
        idx = rotSecTime >= xMin & rotSecTime <= xMax;

        % Compute input between points
        in = [rotors.meas_v0(valid)';
              rotors.meas_v1(valid)';
              rotors.meas_v2(valid)';
              rotors.meas_v3(valid)';
              rotors.meas_v4(valid)';
              rotors.meas_v5(valid)';
              rotors.meas_v6(valid)';
              rotors.meas_v7(valid)';];
        in = mean(in(:,idx),2);
        measIn = sign(in) .* in.^2;

        % Extract rotorcraft data in range
        idx = FTSecTime >= xMin & FTSecTime <= xMax;

        % Compute mean wrench between points
        measWrench = [mean(bodWrench(1,idx));
                      mean(bodWrench(2,idx));
                      mean(bodWrench(3,idx));
                      mean(bodWrench(4,idx));
                      mean(bodWrench(5,idx));
                      mean(bodWrench(6,idx));];

        % Plot selected points
        yline(ax2,measWrench(1), '--r', 'LineWidth', lineSize, 'DisplayName' ,strcat('$$','f_{x}^{meas}','$$'))
        yline(ax2,measWrench(2), '--g', 'LineWidth', lineSize, 'DisplayName' ,strcat('$$','f_{y}^{meas}','$$'))
        yline(ax2,measWrench(3), '--b', 'LineWidth', lineSize, 'DisplayName' ,strcat('$$','f_{z}^{meas}','$$'))
        yline(ax3,measWrench(4), '--r', 'LineWidth', lineSize, 'DisplayName' ,strcat('$$','m_{x}^{meas}','$$'))
        yline(ax3,measWrench(5), '--g', 'LineWidth', lineSize, 'DisplayName' ,strcat('$$','m_{y}^{meas}','$$'))
        yline(ax3,measWrench(6), '--b', 'LineWidth', lineSize, 'DisplayName' ,strcat('$$','m_{z}^{meas}','$$'))

        % Save datapoint in struct
        datName = strcat('dat_',num2str(logChoice),'_',num2str(datNum));
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
    saveas(fig,strcat(figFolder,'dat_',num2str(logChoice),'.png'));    
end

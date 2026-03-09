clear,clc,close all



%% Choose session(s) to plot
forceViolinData = [];
torqueViolinData = [];
forceGroupData = [];
torqueGroupData = [];
colourGroupData = [];
le = [];
i=1;

% visualization parameters
lineSize = 2;
legFontSize = 20;

while true
    % Select data to analyze
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

    % Extract data from logs
    extractLog(strcat(logFolder, 'phyn.log'),'phynt');
    extractLog(strcat(logFolder, 'stats.log'),'pom');
    extractLog(strcat(logFolder, 'omni.log'),'omni');

    % Extract times
    globalStart = min([pom.ts(1), phynt.ts(1), omni.ts(1)]);
    pomTime   = pom.ts   - globalStart;
    phyntTime = phynt.ts - globalStart;
    omniTime = omni.ts - globalStart;

    % Plot body position
    fig = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
    set(fig,'defaulttextinterpreter','latex');
    ax1 = subplot(3,1,1);
    hold on
    grid on
    plot(pomTime,pom.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{B}','$$'));
    plot(pomTime,pom.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{B}','$$'));
    plot(pomTime,pom.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{B}','$$'));
    
    plot(omniTime,omni.xd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{B}','$$'));
    plot(omniTime,omni.yd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{B}','$$'));
    plot(omniTime,omni.zd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{B}','$$'));
    
    leg = legend('show','Location','NorthEastOutside');
    set(leg,'Interpreter','latex','FontSize',legFontSize);
    set(gca,'FontSize',legFontSize-10);
    xlabel('time [sec]','FontSize',legFontSize)
    ylabel('pos [meters]','FontSize',legFontSize)
    title('UAV body (CoM) position Vs time','FontSize',legFontSize)
    
    ax2 = subplot(3,1,2);
    hold on
    grid on
    plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
    plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
    plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));
    
    plot(omniTime,rad2deg(unwrap(omni.rolld)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{B}','$$'));
    plot(omniTime,rad2deg(unwrap(omni.pitchd)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{B}','$$'));
    plot(omniTime,rad2deg(unwrap(omni.yawd)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{B}','$$'));

    leg = legend('show','Location','NorthEastOutside');
    set(leg,'Interpreter','latex','FontSize',legFontSize);
    set(gca,'FontSize',legFontSize-10);
    xlabel('time [sec]','FontSize',legFontSize)
    ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize)
    title('UAV body orientation vs time','FontSize',legFontSize)

    ax3 = subplot(3,1,3);
    hold on
    grid on
    plot(phyntTime,phynt.efx,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{B_{x}}','$$'));
    plot(phyntTime,phynt.efy,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{B_{y}}','$$'));
    plot(phyntTime,phynt.efz,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{B_{z}}','$$'));
    leg = legend('show','Location','NorthEastOutside');
    set(leg,'Interpreter','latex','FontSize',legFontSize);
    set(gca,'FontSize',legFontSize-10);
    xlabel('time [sec]','FontSize',legFontSize)
    ylabel('Force [N]','FontSize',legFontSize)
    title('Estimated force on body and end-effector Vs time','FontSize',legFontSize)

    linkaxes([ax1, ax2, ax3], 'x');
    
    % Let user click two points on the x-axis
    disp('Select two points to define time range');    
    [xRange, ~] = ginput(2);
        
    xMin = min(xRange);
    xMax = max(xRange);
        
    % Add vertical lines to show selection
    xline(xMin, 'r--', 'LineWidth', lineSize, 'HandleVisibility', 'off');
    xline(xMax, 'r--', 'LineWidth', lineSize, 'HandleVisibility', 'off');

    % Extract rotorcraft data in range
    idx = phyntTime >= xMin & phyntTime <= xMax;

    % Collect data for violin plot
    N = numel(phynt.efx(idx));
    forceViolinData = [forceViolinData; phynt.efx(idx);phynt.efy(idx);phynt.efz(idx)];
    forceGroupData = [forceGroupData;categorical(repelem(["$$\hat{f}_{B_{x}}$$";"$$\hat{f}_{B_{y}}$$";"$$\hat{f}_{B_{z}}$$"],[N;N;N]))];
    colourGroupData = [colourGroupData; categorical(repelem(string(char('aaa' + i)'), N))];

    torqueViolinData = [torqueViolinData; phynt.etx(idx);phynt.ety(idx);phynt.etz(idx)];
    torqueGroupData = [torqueGroupData;categorical(repelem(["$$\hat{m}_{B_{x}}$$";"$$\hat{m}_{B_{y}}$$";"$$\hat{m}_{B_{z}}$$"],[N;N;N]))];

    saveas(fig,strcat(figFolder,'viol.png'));

    % Ask if the user wants to continue
    answer = questdlg('Do you want to compare more datasets?', ...
                      'Continue?', ...
                      'Yes', 'No', 'No');
           
    if strcmp(answer, 'No')
        break;
    end    
    i=i+1;
end

%% Violin plot wrench estimate
saveFolder = '/home/rveenstra/MTP/data/Fin/Identification/Hovering/Region3/';

fig = figure('name','Violin plots', 'Position', [1 1 1920 1080]);
set(fig,'defaulttextinterpreter','latex');
subplot(2,1,1)
grid on
violinplot(forceGroupData,forceViolinData,GroupByColor=colourGroupData);
leg = legend("Nominal","$$\Delta = 3 \cdot 10^{-5}$$","$$\Delta = 5 \cdot 10^{-5}$$","$$\Delta = 1 \cdot 10^{-4}$$",'show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'TickLabelInterpreter', 'latex','FontSize',legFontSize);
ylabel("Force estimate error [N]",'FontSize',legFontSize)
title('Force estimator violin plots','FontSize',legFontSize)

subplot(2,1,2)
grid on
violinplot(torqueGroupData,torqueViolinData,GroupByColor=colourGroupData);
leg = legend("Nominal","$$\Delta = 3 \cdot 10^{-5}$$","$$\Delta = 5 \cdot 10^{-5}$$","$$\Delta = 1 \cdot 10^{-4}$$",'show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'TickLabelInterpreter', 'latex','FontSize',legFontSize);
ylabel("Torque estimate error [Nm]",'FontSize',legFontSize)
title('Torque estimator violin plots','FontSize',legFontSize)

%f = gcf;
%exportgraphics(gcf,strcat(saveFolder,'violin.pdf'),'ContentType','vector')

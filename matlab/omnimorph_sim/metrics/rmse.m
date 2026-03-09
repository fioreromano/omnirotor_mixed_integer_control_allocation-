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
extractLog(strcat(logFolder, 'omni.log'),'omni');
extractLog(strcat(logFolder, 'phyn.log'),'phynt');
extractLog(strcat(logFolder, 'stats.log'),'pom');

globalStart = min([phynt.ts(1), omni.ts(1), pom.ts(1)]);
omniTime   = omni.ts   - globalStart;
phyntTime   = phynt.ts   - globalStart;
pomTime   = pom.ts   - globalStart;

%% Compute end effector error
% Extract end-effector position and velocity from Pom
N = numel(pom.ts);
pomEndEffector = pom;

for i = 1:N
    % Extract roll, pitch, yaw and position
    eul = [pomEndEffector.yaw(i), pomEndEffector.pitch(i), pomEndEffector.roll(i)];  % [Z Y X]
    
    % Compute rotation matrix
    R = eul2rotm(eul, 'ZYX');
    
    % Add to the world-frame position
    posOffset = R * config.uavParams.endEffector.p_e;
    pomEndEffector.x(i) = pomEndEffector.x(i) + posOffset(1);
    pomEndEffector.y(i) = pomEndEffector.y(i) + posOffset(2);
    pomEndEffector.z(i) = pomEndEffector.z(i) + posOffset(3);

    velOffset = skewMat([pom.wx(i); pom.wy(i); pom.wz(i)]) * R * config.uavParams.endEffector.p_e;
    pomEndEffector.vx(i) = pomEndEffector.vx(i) + velOffset(1);
    pomEndEffector.vy(i) = pomEndEffector.vy(i) + velOffset(2);
    pomEndEffector.vz(i) = pomEndEffector.vz(i) + velOffset(3);   

    eulEndEffector = rotm2eul(R*config.uavParams.endEffector.R_e, 'ZYX');
    pomEndEffector.yaw(i) = eulEndEffector(1);
    pomEndEffector.pitch(i) = eulEndEffector(2); 
    pomEndEffector.roll(i) = eulEndEffector(3); 
end

% Interpolate phynt signals onto pomTime grid
phyntX_interp = interp1(phyntTime, phynt.x, pomTime, 'linear', 'extrap');
phyntY_interp = interp1(phyntTime, phynt.y, pomTime, 'linear', 'extrap');
phyntZ_interp = interp1(phyntTime, phynt.z, pomTime, 'linear', 'extrap');

% Difference
diffX = pomEndEffector.x - phyntX_interp;
diffY = pomEndEffector.y - phyntY_interp;
diffZ = pomEndEffector.z - phyntZ_interp;

%% Plot UAV body and end effector error
% Visualization parameters
lineSize = 2;
legFontSize = 20;

fig = figure('name','UAV Error', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
ax1 = subplot(4,1,1);
hold on
grid on
plot(phyntTime,phynt.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{r}_{E}','$$'));
plot(phyntTime,phynt.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{r}_{E}','$$'));
plot(phyntTime,phynt.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{r}_{E}','$$')); 

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-5);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('pos [meters]','FontSize',legFontSize-5)
title('UAV end-effector position Vs time','FontSize',legFontSize)

ax2 = subplot(4,1,2);
hold on
grid on
plot(omniTime,omni.e_x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
plot(omniTime,omni.e_y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
plot(omniTime,omni.e_z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-5);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('pos [meters]','FontSize',legFontSize-5)
title('UAV body (CoM) position error Vs time','FontSize',legFontSize)

ax3 = subplot(4,1,3);
hold on
grid on
plot(pomTime,diffX,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
plot(pomTime,diffY,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
plot(pomTime,diffZ,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-5);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('pos [meters]','FontSize',legFontSize-5)
title('UAV end-effector position error Vs time','FontSize',legFontSize)

ax4 = subplot(4,1,4);
hold on
grid on
plot(omniTime,rad2deg(omni.e_rx), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\phi}','$$')); 
plot(omniTime,rad2deg(omni.e_ry), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\theta}','$$')); 
plot(omniTime,rad2deg(omni.e_rz), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\psi}','$$')); 

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-5);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize-5)
title('UAV body orientation error vs time','FontSize',legFontSize)

linkaxes([ax1,ax2,ax3,ax4],'x')

% Let user click two points on the x-axis
disp('Select two points to define time range');
[xRange, ~] = ginput(2);

xMin = min(xRange);
xMax = max(xRange);

% Add vertical lines to show selection
xline(xMin, 'r--', 'LineWidth', 1.5,'HandleVisibility', 'off');
xline(xMax, 'r--', 'LineWidth', 1.5,'HandleVisibility', 'off');

saveas(fig,strcat(figFolder,'rmse.png'));

% Extract data in range
idx = omniTime >= xMin & omniTime <= xMax;
timeSel = omniTime(idx);
exSel = omni.e_x(idx);
eySel = omni.e_y(idx);
ezSel = omni.e_z(idx);
erxSel = omni.e_rx(idx);
erySel = omni.e_ry(idx);
erzSel = omni.e_rz(idx);

% Compute acceleration and velocity magnitude
dataStruct.maxBodAcc = [max(sqrt(omni.axd(idx).^2 + omni.ayd(idx).^2 + omni.azd(idx).^2))];
dataStruct.maxBodAngVel = [max(sqrt(omni.wxd(idx).^2 + omni.wyd(idx).^2 + omni.wzd(idx).^2))];

idx = phyntTime >= xMin & phyntTime <= xMax;
dataStruct.maxEeAcc = [max(sqrt(phynt.ax(idx).^2 + phynt.ay(idx).^2 + phynt.az(idx).^2))];
dataStruct.maxEeAngVel = [max(sqrt(phynt.wx(idx).^2 + phynt.wy(idx).^2 + phynt.wz(idx).^2))];

idx = pomTime >= xMin & pomTime <= xMax;
exEeSel = diffX(idx);
eyEeSel = diffY(idx);
ezEeSel = diffZ(idx);

% Compute RMSE
dataStruct.bodPosRMSE = [sqrt(1/(numel(exSel)) * sum(exSel.^2));
                         sqrt(1/(numel(eySel)) * sum(eySel.^2));
                         sqrt(1/(numel(ezSel)) * sum(ezSel.^2))];

dataStruct.OrRMSE = [rad2deg(sqrt(1/(numel(erxSel)) * sum(erxSel.^2)));
                     rad2deg(sqrt(1/(numel(erySel)) * sum(erySel.^2)));
                     rad2deg(sqrt(1/(numel(erzSel)) * sum(erzSel.^2)))];

dataStruct.eePosRMSE = [sqrt(1/(numel(exEeSel)) * sum(exEeSel.^2));
                        sqrt(1/(numel(eyEeSel)) * sum(eyEeSel.^2));
                        sqrt(1/(numel(ezEeSel)) * sum(ezEeSel.^2))];

% Compute max
dataStruct.eePosMax = [max(abs(exEeSel));
                        max(abs(eyEeSel));
                        max(abs(ezEeSel))];

dataStruct.bodPosMax = [max(abs(exSel));
                       max(abs(eySel));
                       max(abs(ezSel))];

dataStruct.bodOrMax = [rad2deg(max(abs(erxSel)));
                       rad2deg(max(abs(erySel)));
                       rad2deg(max(abs(erzSel)))];

save(strcat(baseFolder,'metrics','.mat'),'dataStruct');

%% Bar plot - RMSE
% Collect data first
accVals = [];
velVals = [];
eeRmseVals = [];
eeMaxVals = [];
orRmseVals = [];
orMaxVals = [];
groups = {};

i = 1;
while true
    baseFolder = uigetdir(pwd, 'Select folder containing data files');
    if isequal(baseFolder, 0)
        error('No folder selected. Aborting log extraction.');
    end
    % Ensure trailing slash
    if baseFolder(end) ~= filesep
        baseFolder = [baseFolder filesep];
    end
    
    load(strcat(baseFolder,'metrics.mat'));

    folders = split(baseFolder,'/');

    accVals(end+1) = dataStruct.maxBodAcc;
    velVals(end+1) = dataStruct.maxBodAngVel;
    eeRmseVals(end+1,:) = dataStruct.eePosRMSE(:)';
    orRmseVals(end+1,:) = dataStruct.OrRMSE(:)';

    eeMaxVals(end+1,:) = dataStruct.eePosMax(:)';
    orMaxVals(end+1,:) = dataStruct.bodOrMax(:)';
        
    groups(end+1) = folders(end-1);
    
    % Ask if the user wants to continue
    answer = questdlg('Do you want to select more flights?', ...
                      'Continue?', ...
                      'Yes', 'No', 'No');
        
    if strcmp(answer, 'No')
        break;
    end

    i = i + 1;
end

%% free flight rotations/CTR RMSE bar plot
saveFolder = '/home/rveenstra/MTP/data/Fin/FF/';

% visualization parameters
figSize = [100,100,1400,600];
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;

%fig = figure('name','RMSE Bar plots', 'Position', [1 1 1920 1080]);
fig = figure('Position',figSize);
tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'RMSE','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups(1:3), eeRmseVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
bar(groups(4:6), eeRmseVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);

ax3 = nexttile; % Move to the next tile
bar(groups(7:9), eeRmseVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax4 = nexttile; % Move to the next tile
bar(groups(1:3), orRmseVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)

ax5 = nexttile; % Move to the next tile
bar(groups(4:6), orRmseVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);

ax6 = nexttile; % Move to the next tile
bar(groups(7:9), orRmseVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'rms_ctr.pdf'),'ContentType','vector')

fig = figure('Position',figSize);
tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'Max error','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups(1:3), eeMaxVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
bar(groups(4:6), eeMaxVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);

ax3 = nexttile; % Move to the next tile
bar(groups(7:9), eeMaxVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax4 = nexttile; % Move to the next tile
bar(groups(1:3), orMaxVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)

ax5 = nexttile; % Move to the next tile
bar(groups(4:6), orMaxVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);

ax6 = nexttile; % Move to the next tile
bar(groups(7:9), orMaxVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'max_ctr.pdf'),'ContentType','vector')

%% free flight translation RMSE bar plot
saveFolder = '/home/rveenstra/MTP/data/Fin/FF/';

% visualization parameters
figSize = [100,100,1400,600];
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;

%fig = figure('name','RMSE Bar plots', 'Position', [1 1 1920 1080]);
fig = figure('Position',figSize);
tiled = tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'RMSE','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups(1:3), eeRmseVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
bar(groups(4:6), eeRmseVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);

ax3 = nexttile; % Move to the next tile
bar(groups(7:9), eeRmseVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);

ax4 = nexttile; % Move to the next tile
bar(groups(10:12), eeRmseVals(10:12,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax5 = nexttile; % Move to the next tile
bar(groups(1:3), orRmseVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)

ax6 = nexttile; % Move to the next tile
bar(groups(4:6), orRmseVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);

ax7 = nexttile; % Move to the next tile
bar(groups(7:9), orRmseVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);

ax8 = nexttile; % Move to the next tile
bar(groups(10:12), orRmseVals(10:12,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'rms_trans.pdf'),'ContentType','vector')

fig = figure('Position',figSize);
tiled = tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'Max error','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups(1:3), eeMaxVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
bar(groups(4:6), eeMaxVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);

ax3 = nexttile; % Move to the next tile
bar(groups(7:9), eeMaxVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);

ax4 = nexttile; % Move to the next tile
bar(groups(10:12), eeMaxVals(10:12,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax5 = nexttile; % Move to the next tile
bar(groups(1:3), orMaxVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)

ax6 = nexttile; % Move to the next tile
bar(groups(4:6), orMaxVals(4:6,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);

ax7 = nexttile; % Move to the next tile
bar(groups(7:9), orMaxVals(7:9,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);

ax8 = nexttile; % Move to the next tile
bar(groups(10:12), orMaxVals(10:12,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'max_trans.pdf'),'ContentType','vector')

%% free flight HR RMSE bar plot
saveFolder = '/home/rveenstra/MTP/data/Fin/FF/';

% visualization parameters
figSize = [100,100,850,600];
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;

%fig = figure('name','RMSE Bar plots', 'Position', [1 1 1920 1080]);
fig = figure('Position',figSize);
tiled = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
%title(tiled,'RMSE','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups(1:3), eeRmseVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)
title("RMSE",'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
bar(groups(1:3), eeMaxVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
title("Max error",'FontSize',labFontSize)
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax3 = nexttile; % Move to the next tile
bar(groups(1:3), orRmseVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)

ax4 = nexttile; % Move to the next tile
bar(groups(1:3), orMaxVals(1:3,:));
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'bar_hr.pdf'),'ContentType','vector')

%% Slide RMSE bar plot
saveFolder = '/home/rveenstra/MTP/data/Fin/SD/';

% visualization parameters
figSize = [100,100,700,600];
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;

%fig = figure('name','RMSE Bar plots', 'Position', [1 1 1920 1080]);
fig = figure('Position',figSize);
tiled = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'RMSE','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups, eeRmseVals);
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax2 = nexttile; % Move to the next tile
bar(groups, orRmseVals);
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'rms_sd.pdf'),'ContentType','vector')

fig = figure('Position',figSize);
tiled = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'Max error','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups, eeMaxVals);
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax2 = nexttile; % Move to the next tile
bar(groups, orMaxVals);
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'max_sd.pdf'),'ContentType','vector')

%% PiH RMSE bar plot
saveFolder = '/home/rveenstra/MTP/data/Fin/PiH/';

% visualization parameters
figSize = [100,100,700,600];
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;

%fig = figure('name','RMSE Bar plots', 'Position', [1 1 1920 1080]);
fig = figure('Position',figSize);
tiled = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,'RMSE','FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
bar(groups, eeRmseVals);
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Position error','[m]'},'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
bar(groups, eeMaxVals);
grid on
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_x','e_y','e_z'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

ax3 = nexttile; % Move to the next tile
bar(groups, orRmseVals);
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'Orientation error','[deg]'},'FontSize',labFontSize)

ax4 = nexttile; % Move to the next tile
bar(groups, orMaxVals);
grid on
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
leg = legend({'e_{\phi}','e_{\theta}','e_{\psi}'});
set(leg,'FontSize',legFontSize,"Location","northeastOutside");

f = gcf;
exportgraphics(gcf,strcat(saveFolder,'max_pih.pdf'),'ContentType','vector')
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

    addpath('../lib/')
    addpath('../lib/omnimorph-helper/')
    addpath('../lib/math-helper/')
    addpath('../sim/')
    addpath('../plot/')

    run('../omnimorph_params.m')
end

%% Extract data
extractLog(strcat(logFolder, 'man.log'),'man');
extractLog(strcat(logFolder, 'phyn.log'),'phynt');
extractLog(strcat(logFolder, 'stats.log'),'pom');
extractLog(strcat(logFolder, 'rot.log'),'rotors');
extractLog(strcat(logFolder, 'omni.log'),'omni');

% Extract times
globalStart = min([pom.ts(1), phynt.ts(1), man.ts(1), rotors.ts(1), omni.ts(1)]);
pomTime   = pom.ts   - globalStart;
phyntTime = phynt.ts - globalStart;
manTime = man.ts - globalStart;
rotTime = rotors.ts - globalStart;
omniTime = omni.ts - globalStart;

globalEnd = max([pomTime(end), phyntTime(end), manTime(end), rotTime(end), omniTime(end)]);

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

valid = ~isnan(rotors.ts) & ~isnan(rotors.meas_v0) & ~isnan(rotors.meas_v1) & ~isnan(rotors.meas_v2) & ~isnan(rotors.meas_v3) & ~isnan(rotors.meas_v4) & ~isnan(rotors.meas_v5) & ~isnan(rotors.meas_v6) & ~isnan(rotors.meas_v7); % Take out NAN values
rotMeasTime = rotTime(valid);

maxPropellerMeas = ones(1,numel(rotMeasTime))*config.uavParams.maxPropSpeed;
minPropellerMeas = ones(1,numel(rotMeasTime))*30;

%% UAV state plots
% visualization parameters
figSize = [100,100,800,800];
lineSize = 1.5;
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;
xMin = 8;
xMax = 2;

fig = figure('Position',figSize); % [left bottom width height]
tiled = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');

ax1 = nexttile; % Move to the next tile
hold on;
grid on;
plot(pomTime,pomEndEffector.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{E}','$$'));
plot(pomTime,pomEndEffector.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{E}','$$'));
plot(pomTime,pomEndEffector.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{E}','$$'));
    
plot(phyntTime,phynt.x,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{r}_{E}','$$'));
plot(phyntTime,phynt.y,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{r}_{E}','$$'));
plot(phyntTime,phynt.z,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{r}_{E}','$$'));    

plot(manTime,man.x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{E}','$$'));
plot(manTime,man.y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{E}','$$'));
plot(manTime,man.z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{E}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal");
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'position','[m]'},'FontSize',labFontSize)
xlim([xMin, globalEnd]) 
yl = ylim;                       
ylim([yl(1)-0.2*range(yl), yl(2)+0.25*range(yl)]);   


ax2 = nexttile; % Move to the next tile
hold on;
grid on;
plot(pomTime,diffX,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
plot(pomTime,diffY,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
plot(pomTime,diffZ,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal");
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'position error','[m]'},'FontSize',labFontSize)
xlim([xMin, globalEnd]) 

ax3 = nexttile; % Move to the next tile
hold on
grid on
plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));

plot(phyntTime,rad2deg(unwrap(phynt.roll)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{r}_{E}','$$'));
plot(phyntTime,rad2deg(unwrap(phynt.pitch)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{r}_{E}','$$'));
plot(phyntTime,rad2deg(unwrap(phynt.yaw)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{r}_{E}','$$'));

plot(manTime,rad2deg(unwrap(man.roll)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{E}','$$'));
plot(manTime,rad2deg(unwrap(man.pitch)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{E}','$$'));
plot(manTime,rad2deg(unwrap(man.yaw)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{E}','$$'));    

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal");
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'orientation','[deg]'},'FontSize',labFontSize)
xlim([xMin, globalEnd]) 
yl = ylim;                      
ylim([yl(1)-0.1*range(yl), yl(2)+0.35*range(yl)]);  

ax4 = nexttile; % Move to the next tile
hold on
grid on
plot(omniTime,rad2deg(omni.e_rx), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\phi}','$$')); 
plot(omniTime,rad2deg(omni.e_ry), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\theta}','$$')); 
plot(omniTime,rad2deg(omni.e_rz), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\psi}','$$')); 

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal");
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel({'orientation error','[deg]'},'FontSize',labFontSize)
xlim([xMin, globalEnd]) 
%yl = ylim;                      
%ylim([yl(1), yl(2)+0.3*range(yl)]);  


xlabel('time [sec]','FontSize',labFontSize)
f = gcf;
exportgraphics(gcf,strcat(figFolder,'states1.pdf'),'ContentType','image')

%% Free flight position plots
% visualization parameters
folders = split(baseFolder,'/');
flightName = folders(end-1);

figSize = [100,100,1800,600];
lineSize = 1.5;
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;
xMin = 8;
xMax = 2;
iconWidth = 15;

fig = figure('Position',figSize); % [left bottom width height]
tiled = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,flightName,'FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,(rotors.cmd_v0(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
plot(rotMeasTime,(rotors.cmd_v1(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
plot(rotMeasTime,(rotors.cmd_v2(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
plot(rotMeasTime,(rotors.cmd_v3(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
plot(rotMeasTime,(rotors.cmd_v4(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
plot(rotMeasTime,(rotors.cmd_v5(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
plot(rotMeasTime,(rotors.cmd_v6(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
plot(rotMeasTime,(rotors.cmd_v7(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));

plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,-maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,-minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Propeller speeds",'FontSize',labFontSize)


ax2 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,(rotors.cmd_v0(valid) - rotors.meas_v0(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
plot(rotMeasTime,(rotors.cmd_v1(valid) - rotors.meas_v1(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
plot(rotMeasTime,(rotors.cmd_v2(valid) - rotors.meas_v2(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
plot(rotMeasTime,(rotors.cmd_v3(valid) - rotors.meas_v3(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
plot(rotMeasTime,(rotors.cmd_v4(valid) - rotors.meas_v4(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
plot(rotMeasTime,(rotors.cmd_v5(valid) - rotors.meas_v5(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
plot(rotMeasTime,(rotors.cmd_v6(valid) - rotors.meas_v6(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
plot(rotMeasTime,(rotors.cmd_v7(valid) - rotors.meas_v7(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Propeller speed error','FontSize',labFontSize)

f = gcf;
exportgraphics(gcf,strcat(figFolder,'prop_speeds.pdf'),'ContentType','image')

%% Free flight position plots
% visualization parameters
folders = split(baseFolder,'/');
flightName = folders(end-1);

figSize = [100,100,1800,600];
lineSize = 1.5;
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;
xMin = 8;
xMax = 2;
iconWidth = 15;

fig = figure('Position',figSize); % [left bottom width height]
tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,flightName,'FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
hold on;
grid on;
plot(pomTime,pomEndEffector.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{E}','$$'));
plot(pomTime,pomEndEffector.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{E}','$$'));
plot(pomTime,pomEndEffector.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{E}','$$'));
    
plot(phyntTime,phynt.x,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{r}_{E}','$$'));
plot(phyntTime,phynt.y,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{r}_{E}','$$'));
plot(phyntTime,phynt.z,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{r}_{E}','$$'));    

plot(manTime,man.x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{E}','$$'));
plot(manTime,man.y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{E}','$$'));
plot(manTime,man.z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{E}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal");
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[m]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Position",'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
hold on
grid on
plot(pomTime,pomEndEffector.vx,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}_{E}','$$'));
plot(pomTime,pomEndEffector.vy,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}_{E}','$$'));
plot(pomTime,pomEndEffector.vz,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}_{E}','$$'));
    
plot(phyntTime,phynt.vx,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{r}_{E}','$$'));
plot(phyntTime,phynt.vy,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{r}_{E}','$$'));
plot(phyntTime,phynt.vz,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{r}_{E}','$$'));    

plot(manTime,man.vx,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{d}_{E}','$$'));
plot(manTime,man.vy,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{d}_{E}','$$'));
plot(manTime,man.vz,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{d}_{E}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[m/s]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Velocity",'FontSize',labFontSize)

ax3 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,rotors.meas_v0(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotMeasTime,rotors.meas_v1(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotMeasTime,rotors.meas_v2(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotMeasTime,rotors.meas_v3(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotMeasTime,rotors.meas_v4(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotMeasTime,rotors.meas_v5(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotMeasTime,rotors.meas_v6(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotMeasTime,rotors.meas_v7(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Propeller speeds",'FontSize',labFontSize)

ax4 = nexttile; % Move to the next tile
hold on
grid on
plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));

plot(phyntTime,rad2deg(unwrap(phynt.roll)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{r}_{E}','$$'));
plot(phyntTime,rad2deg(unwrap(phynt.pitch)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{r}_{E}','$$'));
plot(phyntTime,rad2deg(unwrap(phynt.yaw)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{r}_{E}','$$'));

plot(manTime,rad2deg(unwrap(man.roll)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{E}','$$'));
plot(manTime,rad2deg(unwrap(man.pitch)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{E}','$$'));
plot(manTime,rad2deg(unwrap(man.yaw)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{E}','$$'));  

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("roll-pitch-yaw",'FontSize',labFontSize)

ax5 = nexttile; % Move to the next tile
hold on
grid on
plot(pomTime,rad2deg(pomEndEffector.wx),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}_{E}','$$'));
plot(pomTime,rad2deg(pomEndEffector.wy),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}_{E}','$$'));
plot(pomTime,rad2deg(pomEndEffector.wz),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}_{E}','$$'));
    
plot(phyntTime,rad2deg(phynt.wx),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{r}_{E}','$$'));
plot(phyntTime,rad2deg(phynt.wy),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{r}_{E}','$$'));
plot(phyntTime,rad2deg(phynt.wz),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{r}_{E}','$$'));

plot(manTime,rad2deg(man.wx),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{d}_{E}','$$'));
plot(manTime,rad2deg(man.wy),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{d}_{E}','$$'));
plot(manTime,rad2deg(man.wz),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{d}_{E}','$$'));   

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg/s]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]); 
title('Angular velocity','FontSize',labFontSize)
xlabel('Time [sec]','FontSize',labFontSize)

ax6 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,(rotors.cmd_v0(valid) - rotors.meas_v0(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
plot(rotMeasTime,(rotors.cmd_v1(valid) - rotors.meas_v1(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
plot(rotMeasTime,(rotors.cmd_v2(valid) - rotors.meas_v2(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
plot(rotMeasTime,(rotors.cmd_v3(valid) - rotors.meas_v3(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
plot(rotMeasTime,(rotors.cmd_v4(valid) - rotors.meas_v4(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
plot(rotMeasTime,(rotors.cmd_v5(valid) - rotors.meas_v5(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
plot(rotMeasTime,(rotors.cmd_v6(valid) - rotors.meas_v6(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
plot(rotMeasTime,(rotors.cmd_v7(valid) - rotors.meas_v7(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Propeller speed error','FontSize',labFontSize)

f = gcf;
exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','image')

%% Free flight error plots
% visualization parameters
folders = split(baseFolder,'/');
flightName = folders(end-1);

figSize = [100,100,1800,600];
lineSize = 1.5;
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;
xMin = 8;
xMax = 2;
iconWidth = 15;

fig = figure('Position',figSize); % [left bottom width height]
tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
set(tiled,'defaulttextinterpreter','latex');
title(tiled,flightName,'FontSize',labFontSize','Interpreter','latex')

ax1 = nexttile; % Move to the next tile
hold on;
grid on;
plot(pomTime,diffX,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
plot(pomTime,diffY,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
plot(pomTime,diffZ,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[m]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Position error",'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
hold on
grid on
plot(pomTime,pomEndEffector.vx,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}_{E}','$$'));
plot(pomTime,pomEndEffector.vy,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}_{E}','$$'));
plot(pomTime,pomEndEffector.vz,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}_{E}','$$'));
    
plot(phyntTime,phynt.vx,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{r}_{E}','$$'));
plot(phyntTime,phynt.vy,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{r}_{E}','$$'));
plot(phyntTime,phynt.vz,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{r}_{E}','$$'));    

plot(manTime,man.vx,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{d}_{E}','$$'));
plot(manTime,man.vy,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{d}_{E}','$$'));
plot(manTime,man.vz,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{d}_{E}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[m/s]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Velocity",'FontSize',labFontSize)

ax3 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,rotors.meas_v0(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotMeasTime,rotors.meas_v1(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotMeasTime,rotors.meas_v2(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotMeasTime,rotors.meas_v3(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotMeasTime,rotors.meas_v4(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotMeasTime,rotors.meas_v5(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotMeasTime,rotors.meas_v6(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotMeasTime,rotors.meas_v7(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Propeller speeds",'FontSize',labFontSize)

ax4 = nexttile; % Move to the next tile
hold on
grid on
plot(omniTime,rad2deg(omni.e_rx), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\phi}','$$')); 
plot(omniTime,rad2deg(omni.e_ry), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\theta}','$$')); 
plot(omniTime,rad2deg(omni.e_rz), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\psi}','$$')); 

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("roll-pitch-yaw error",'FontSize',labFontSize)

ax5 = nexttile; % Move to the next tile
hold on
grid on
plot(pomTime,rad2deg(pomEndEffector.wx),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}_{E}','$$'));
plot(pomTime,rad2deg(pomEndEffector.wy),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}_{E}','$$'));
plot(pomTime,rad2deg(pomEndEffector.wz),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}_{E}','$$'));
    
plot(phyntTime,rad2deg(phynt.wx),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{r}_{E}','$$'));
plot(phyntTime,rad2deg(phynt.wy),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{r}_{E}','$$'));
plot(phyntTime,rad2deg(phynt.wz),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{r}_{E}','$$'));

plot(manTime,rad2deg(man.wx),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{d}_{E}','$$'));
plot(manTime,rad2deg(man.wy),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{d}_{E}','$$'));
plot(manTime,rad2deg(man.wz),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{d}_{E}','$$'));   

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg/s]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]); 
title('Angular velocity','FontSize',labFontSize)
xlabel('Time [sec]','FontSize',labFontSize)

ax6 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,(rotors.cmd_v0(valid) - rotors.meas_v0(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
plot(rotMeasTime,(rotors.cmd_v1(valid) - rotors.meas_v1(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
plot(rotMeasTime,(rotors.cmd_v2(valid) - rotors.meas_v2(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
plot(rotMeasTime,(rotors.cmd_v3(valid) - rotors.meas_v3(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
plot(rotMeasTime,(rotors.cmd_v4(valid) - rotors.meas_v4(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
plot(rotMeasTime,(rotors.cmd_v5(valid) - rotors.meas_v5(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
plot(rotMeasTime,(rotors.cmd_v6(valid) - rotors.meas_v6(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
plot(rotMeasTime,(rotors.cmd_v7(valid) - rotors.meas_v7(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Propeller speed error','FontSize',labFontSize)

f = gcf;
exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','image')

%% Sliding/PiH - velocity
% % visualization parameters
% folders = split(baseFolder,'/');
% flightName = folders(end-1);
% 
% figSize = [100,100,1800,600];
% lineSize = 1.5;
% legFontSize = 12;
% labFontSize = 15;
% tickSize = 10;
% gridSize = 1.5;
% xMin = 8;
% xMax = 2;
% iconWidth = 15;
% 
% fig = figure('Position',figSize); % [left bottom width height]
% tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
% set(tiled,'defaulttextinterpreter','latex');
% title(tiled,flightName,'FontSize',labFontSize','Interpreter','latex')
% 
% ax1 = nexttile; % Move to the next tile
% hold on;
% grid on;
% plot(pomTime,pomEndEffector.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{E}','$$'));
% plot(pomTime,pomEndEffector.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{E}','$$'));
% plot(pomTime,pomEndEffector.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{E}','$$'));
% 
% plot(phyntTime,phynt.x,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{r}_{E}','$$'));
% plot(phyntTime,phynt.y,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{r}_{E}','$$'));
% plot(phyntTime,phynt.z,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{r}_{E}','$$'));    
% 
% plot(manTime,man.x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{E}','$$'));
% plot(manTime,man.y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{E}','$$'));
% plot(manTime,man.z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{E}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Position",'FontSize',labFontSize)
% 
% ax2 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,pomEndEffector.vx,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}_{E}','$$'));
% plot(pomTime,pomEndEffector.vy,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}_{E}','$$'));
% plot(pomTime,pomEndEffector.vz,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}_{E}','$$'));
% 
% plot(phyntTime,phynt.vx,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{r}_{E}','$$'));
% plot(phyntTime,phynt.vy,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{r}_{E}','$$'));
% plot(phyntTime,phynt.vz,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{r}_{E}','$$'));    
% 
% plot(manTime,man.vx,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{d}_{E}','$$'));
% plot(manTime,man.vy,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{d}_{E}','$$'));
% plot(manTime,man.vz,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{d}_{E}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m/s]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Velocity",'FontSize',labFontSize)
% 
% ax3 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1500),phynt.efx(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efy(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efz(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[N]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Estimated force",'FontSize',labFontSize)
% 
% ax4 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
% plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
% plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));
% 
% plot(phyntTime,rad2deg(unwrap(phynt.roll)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(unwrap(phynt.pitch)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(unwrap(phynt.yaw)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{r}_{E}','$$'));
% 
% plot(manTime,rad2deg(unwrap(man.roll)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{E}','$$'));
% plot(manTime,rad2deg(unwrap(man.pitch)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{E}','$$'));
% plot(manTime,rad2deg(unwrap(man.yaw)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{E}','$$'));  
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("roll-pitch-yaw",'FontSize',labFontSize)
% 
% ax5 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,rad2deg(pomEndEffector.wx),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}_{E}','$$'));
% plot(pomTime,rad2deg(pomEndEffector.wy),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}_{E}','$$'));
% plot(pomTime,rad2deg(pomEndEffector.wz),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}_{E}','$$'));
% 
% plot(phyntTime,rad2deg(phynt.wx),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(phynt.wy),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(phynt.wz),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{r}_{E}','$$'));
% 
% plot(manTime,rad2deg(man.wx),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{d}_{E}','$$'));
% plot(manTime,rad2deg(man.wy),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{d}_{E}','$$'));
% plot(manTime,rad2deg(man.wz),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{d}_{E}','$$'));   
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg/s]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]); 
% title('Angular velocity','FontSize',labFontSize)
% xlabel('Time [sec]','FontSize',labFontSize)
% 
% ax6 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.etx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.ety(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.etz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Nm]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Estimated torque','FontSize',labFontSize)
% 
%f = gcf;
%exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','vector')

%% Sliding - wrenches

% N = numel(phynt.ts);
% 
% for i = 1:N
%     % Extract roll, pitch, yaw and position
%     eul = [phynt.yaw(i), phynt.pitch(i), phynt.roll(i)];  % [Z Y X]
% 
%     % Compute rotation matrix (assumes desired approximately equal to
%     % actual orientation)
%     R = eul2rotm(eul, 'ZYX');
% 
%     % Add to the world-frame position
%     bodForce(:,i) = R' * [phynt.efx(i); phynt.efy(i); phynt.efz(i)];
%     bodTorque(:,i) = R' * [phynt.etx(i); phynt.ety(i); phynt.etz(i)];
% end
% 
% % visualization parameters
% folders = split(baseFolder,'/');
% flightName = folders(end-1);
% 
% figSize = [100,100,1800,600];
% lineSize = 1.5;
% legFontSize = 12;
% labFontSize = 15;
% tickSize = 10;
% gridSize = 1.5;
% xMin = 8;
% xMax = 2;
% iconWidth = 15;
% 
% fig = figure('Position',figSize); % [left bottom width height]
% tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
% set(tiled,'defaulttextinterpreter','latex');
% title(tiled,flightName,'FontSize',labFontSize','Interpreter','latex')
% 
% ax1 = nexttile; % Move to the next tile
% hold on;
% grid on;
% plot(pomTime,pomEndEffector.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{E}','$$'));
% plot(pomTime,pomEndEffector.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{E}','$$'));
% plot(pomTime,pomEndEffector.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{E}','$$'));
% 
% plot(phyntTime,phynt.x,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{r}_{E}','$$'));
% plot(phyntTime,phynt.y,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{r}_{E}','$$'));
% plot(phyntTime,phynt.z,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{r}_{E}','$$'));    
% 
% plot(manTime,man.x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{E}','$$'));
% plot(manTime,man.y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{E}','$$'));
% plot(manTime,man.z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{E}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Position",'FontSize',labFontSize)
% 
% ax2 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),bodForce(1,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),bodForce(2,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),bodForce(3,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[N]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Estimated force",'FontSize',labFontSize)
% 
% ax3 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.efx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.efy(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.efz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[N]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Estimated force",'FontSize',labFontSize)
% 
% ax4 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
% plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
% plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));
% 
% plot(phyntTime,rad2deg(unwrap(phynt.roll)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(unwrap(phynt.pitch)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(unwrap(phynt.yaw)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{r}_{E}','$$'));
% 
% plot(manTime,rad2deg(unwrap(man.roll)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{E}','$$'));
% plot(manTime,rad2deg(unwrap(man.pitch)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{E}','$$'));
% plot(manTime,rad2deg(unwrap(man.yaw)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{E}','$$'));  
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("roll-pitch-yaw",'FontSize',labFontSize)
% 
% ax5 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),bodTorque(1,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),bodTorque(2,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),bodTorque(3,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg/s]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]); 
% title('Estimated torque','FontSize',labFontSize)
% xlabel('Time [sec]','FontSize',labFontSize)
% 
% ax6 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.etx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.ety(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.etz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Nm]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Estimated torque','FontSize',labFontSize)

%f = gcf;
%exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','vector')

 %% Optimization - pos
% % visualization parameters
% figSize = [100,100,1800,600];
% lineSize = 1.5;
% legFontSize = 12;
% labFontSize = 15;
% tickSize = 10;
% gridSize = 1.5;
% xMin = 8;
% xMax = 2;
% iconWidth = 15;
% 
% fig = figure('Position',figSize); % [left bottom width height]
% tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
% set(tiled,'defaulttextinterpreter','latex');
% title(tiled,'Tilting with optimized wrench map, $$\Delta = 5 \cdot 10^{-5}$$','FontSize',labFontSize','Interpreter','latex')
% 
% ax1 = nexttile; % Move to the next tile
% hold on;
% grid on;
% plot(pomTime,pom.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{B}','$$'));
% plot(pomTime,pom.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{B}','$$'));
% plot(pomTime,pom.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{B}','$$'));
% 
% plot(omniTime,omni.xd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{B}','$$'));
% plot(omniTime,omni.yd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{B}','$$'));
% plot(omniTime,omni.zd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{B}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Position",'FontSize',labFontSize)
% 
% ax2 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(rotMeasTime,rotors.meas_v0(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
% plot(rotMeasTime,rotors.meas_v1(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
% plot(rotMeasTime,rotors.meas_v2(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
% plot(rotMeasTime,rotors.meas_v3(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
% plot(rotMeasTime,rotors.meas_v4(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
% plot(rotMeasTime,rotors.meas_v5(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
% plot(rotMeasTime,rotors.meas_v6(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
% plot(rotMeasTime,rotors.meas_v7(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));
% 
% plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
% plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Hz]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Propeller speeds",'FontSize',labFontSize)
% 
% ax3 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1500),phynt.efx(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efy(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efz(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[N]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Estimated force",'FontSize',labFontSize)
% 
% ax4 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
% plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
% plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));
% 
% plot(omniTime,rad2deg(unwrap(omni.rolld)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{B}','$$'));
% plot(omniTime,rad2deg(unwrap(omni.pitchd)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{B}','$$'));
% plot(omniTime,rad2deg(unwrap(omni.yawd)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{B}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("roll-pitch-yaw",'FontSize',labFontSize)
% 
% ax5 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(rotMeasTime,(rotors.cmd_v0(valid) - rotors.meas_v0(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v1(valid) - rotors.meas_v1(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v2(valid) - rotors.meas_v2(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v3(valid) - rotors.meas_v3(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v4(valid) - rotors.meas_v4(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v5(valid) - rotors.meas_v5(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v6(valid) - rotors.meas_v6(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v7(valid) - rotors.meas_v7(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Hz]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Propeller speed error','FontSize',labFontSize)
% 
% ax6 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.etx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.ety(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.etz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Nm]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Estimated torque','FontSize',labFontSize)
% 
% %f = gcf;
% %exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','vector')
% 
% 
% %% Optimization - err
% % visualization parameters
% figSize = [100,100,1800,600];
% lineSize = 1.5;
% legFontSize = 12;
% labFontSize = 15;
% tickSize = 10;
% gridSize = 1.5;
% xMin = 8;
% xMax = 2;
% iconWidth = 15;
% 
% fig = figure('Position',figSize); % [left bottom width height]
% tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
% set(tiled,'defaulttextinterpreter','latex');
% title(tiled,'Tilting with optimized wrench map, $$\Delta = 3 \cdot 10^{-5}$$','FontSize',labFontSize','Interpreter','latex')
% 
% ax1 = nexttile; % Move to the next tile
% hold on;
% grid on;
% plot(omniTime,omni.e_x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
% plot(omniTime,omni.e_y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
% plot(omniTime,omni.e_z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Position error",'FontSize',labFontSize)
% 
% ax2 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(rotMeasTime,rotors.meas_v0(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
% plot(rotMeasTime,rotors.meas_v1(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
% plot(rotMeasTime,rotors.meas_v2(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
% plot(rotMeasTime,rotors.meas_v3(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
% plot(rotMeasTime,rotors.meas_v4(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
% plot(rotMeasTime,rotors.meas_v5(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
% plot(rotMeasTime,rotors.meas_v6(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
% plot(rotMeasTime,rotors.meas_v7(valid),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));
% 
% plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
% plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Hz]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Propeller speeds",'FontSize',labFontSize)
% 
% ax3 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1500),phynt.efx(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efy(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efz(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[N]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Estimated force",'FontSize',labFontSize)
% 
% ax4 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(omniTime(1:end-1200),rad2deg(omni.e_rx(1:end-1200)), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\phi}','$$')); 
% plot(omniTime(1:end-1200),rad2deg(omni.e_ry(1:end-1200)), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\theta}','$$')); 
% plot(omniTime(1:end-1200),rad2deg(omni.e_rz(1:end-1200)), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\psi}','$$')); 
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("roll-pitch-yaw error",'FontSize',labFontSize)
% 
% ax5 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(rotMeasTime,(rotors.cmd_v0(valid) - rotors.meas_v0(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v1(valid) - rotors.meas_v1(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v2(valid) - rotors.meas_v2(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v3(valid) - rotors.meas_v3(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v4(valid) - rotors.meas_v4(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v5(valid) - rotors.meas_v5(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v6(valid) - rotors.meas_v6(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
% plot(rotMeasTime,(rotors.cmd_v7(valid) - rotors.meas_v7(valid)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Hz]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Propeller speed error','FontSize',labFontSize)
% 
% ax6 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.etx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.ety(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.etz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Nm]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Estimated torque','FontSize',labFontSize)
% 
% %f = gcf;
% %exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','vector')

%% PiH - velocity
% % visualization parameters
% folders = split(baseFolder,'/');
% flightName = folders(end-1);
% 
% figSize = [100,100,1800,600];
% lineSize = 1.5;
% legFontSize = 12;
% labFontSize = 15;
% tickSize = 10;
% gridSize = 1.5;
% xMin = 8;
% xMax = 2;
% iconWidth = 15;
% 
% fig = figure('Position',figSize); % [left bottom width height]
% tiled = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
% set(tiled,'defaulttextinterpreter','latex');
% title(tiled,flightName,'FontSize',labFontSize','Interpreter','latex')
% 
% ax1 = nexttile; % Move to the next tile
% hold on;
% grid on;
% plot(pomTime,pomEndEffector.x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{E}','$$'));
% plot(pomTime,pomEndEffector.y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{E}','$$'));
% plot(pomTime,pomEndEffector.z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{E}','$$'));
% 
% plot(phyntTime,phynt.x,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{r}_{E}','$$'));
% plot(phyntTime,phynt.y,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{r}_{E}','$$'));
% plot(phyntTime,phynt.z,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{r}_{E}','$$'));    
% 
% plot(manTime,man.x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x^{d}_{E}','$$'));
% plot(manTime,man.y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y^{d}_{E}','$$'));
% plot(manTime,man.z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z^{d}_{E}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Position",'FontSize',labFontSize)
% 
% ax2 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,pomEndEffector.vx,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}_{E}','$$'));
% plot(pomTime,pomEndEffector.vy,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}_{E}','$$'));
% plot(pomTime,pomEndEffector.vz,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}_{E}','$$'));
% 
% plot(phyntTime,phynt.vx,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{r}_{E}','$$'));
% plot(phyntTime,phynt.vy,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{r}_{E}','$$'));
% plot(phyntTime,phynt.vz,'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{r}_{E}','$$'));    
% 
% plot(manTime,man.vx,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{d}_{E}','$$'));
% plot(manTime,man.vy,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{d}_{E}','$$'));
% plot(manTime,man.vz,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{d}_{E}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[m/s]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Velocity",'FontSize',labFontSize)
% 
% ax3 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1500),phynt.efx(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{E_{x}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efy(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{E_{y}}','$$'));
% plot(phyntTime(1:end-1500),phynt.efz(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{E_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[N]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("Estimated force",'FontSize',labFontSize)
% 
% ax4 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,rad2deg(unwrap(pom.roll)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
% plot(pomTime,rad2deg(unwrap(pom.pitch)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
% plot(pomTime,rad2deg(unwrap(pom.yaw)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));
% 
% plot(phyntTime,rad2deg(unwrap(phynt.roll)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(unwrap(phynt.pitch)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(unwrap(phynt.yaw)),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{r}_{E}','$$'));
% 
% plot(manTime,rad2deg(unwrap(man.roll)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi^{d}_{E}','$$'));
% plot(manTime,rad2deg(unwrap(man.pitch)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta^{d}_{E}','$$'));
% plot(manTime,rad2deg(unwrap(man.yaw)),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi^{d}_{E}','$$'));  
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title("roll-pitch-yaw",'FontSize',labFontSize)
% 
% ax5 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(pomTime,rad2deg(pomEndEffector.wx),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}_{E}','$$'));
% plot(pomTime,rad2deg(pomEndEffector.wy),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}_{E}','$$'));
% plot(pomTime,rad2deg(pomEndEffector.wz),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}_{E}','$$'));
% 
% plot(phyntTime,rad2deg(phynt.wx),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(phynt.wy),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{r}_{E}','$$'));
% plot(phyntTime,rad2deg(phynt.wz),'-.','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{r}_{E}','$$'));
% 
% plot(manTime,rad2deg(man.wx),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{d}_{E}','$$'));
% plot(manTime,rad2deg(man.wy),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{d}_{E}','$$'));
% plot(manTime,rad2deg(man.wz),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{d}_{E}','$$'));   
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[deg/s]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]); 
% title('Angular velocity','FontSize',labFontSize)
% xlabel('Time [sec]','FontSize',labFontSize)
% 
% ax6 = nexttile; % Move to the next tile
% hold on
% grid on
% plot(phyntTime(1:end-1500),phynt.eetx(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{E_{x}}','$$'));
% plot(phyntTime(1:end-1500),phynt.eety(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{E_{y}}','$$'));
% plot(phyntTime(1:end-1500),phynt.eetz(1:end-1500),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{E_{z}}','$$'));
% 
% leg = legend('show');
% set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal" );
% set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
% ylabel('[Nm]','FontSize',labFontSize)
% xlim([xMin, globalEnd+xMax]) 
% yl = ylim;                       
% ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
% title('Estimated torque','FontSize',labFontSize)
% 
% f = gcf;
% exportgraphics(gcf,strcat(figFolder,'states2.pdf'),'ContentType','vector')

%% Plot Optimization Results
% distinct_colors = lines(7);
% 
% method_colors = {
%     1, [1 1 1],  'white',   '$pinv$';      
%     2, [1.00 0.65 0.00],  'Blu',     '$P.E$';      
%     3, [0.00 0.60 0.25],  'Verde',   '$P.N$';      
%     4, [0.00 0.35 0.90],  'Arancio', '$QP1$';       
%     5, [0.55 0.00 0.75],  'Viola',   '$T_{\mathrm{P.E}}$';   
%     6, [0.00 0.75 0.75],  'Ciano',   '$T_{\mathrm{P.N}}$';   
%     7, [0.20 0.20 0.20],  'Grigio',  '$QP2$';       
% };   
% yplain = {'pinv','P.E','P.N','QP1','T_{P.E}','T_{P.N}','QP2'};
% n_samples = length(omni.method_used);  
% 
% fig1 = figure('name', 'Controller Methods Timeline', 'Position', [100, 100, 1400, 500]);
% set(fig1, 'defaulttextinterpreter', 'latex');hold on;
% if n_samples > 1
%     method_changes = [1; find(diff(omni.method_used) ~= 0) + 1; n_samples + 1];
% else
%     method_changes = [1; n_samples + 1];
% end
% for interval = 1:(length(method_changes) - 1)
%     start_idx = method_changes(interval);
%     end_idx = method_changes(interval + 1) - 1;        
%     color_idx = omni.method_used(start_idx);
%     if color_idx >= 1 && color_idx <= 7
%         bg_color = method_colors{color_idx, 2};
%         if start_idx == 1
%             t_start = 0;
%         else
%             t_start = omniTime(start_idx);
%         end
% 
%         if (end_idx + 1) <= length(omniTime)
%             t_end = omniTime(end_idx + 1);
%         else
%             t_end = omniTime(end);
%         end            
%         fill([t_start, t_end, t_end, t_start], ...
%              [0, 0, method_colors{color_idx, 1}, method_colors{color_idx, 1}], bg_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
%     end
% end  
% stairs(omniTime(1:min(end, n_samples)), omni.method_used, 'k--','LineWidth', 0.1);
% xlabel('time [sec]');
% ylabel('method');
% title('Controller Methods Timeline', 'FontSize', 12, 'FontWeight', 'bold');
% ylim([0.5, 7.5]);
% yticks(1:7);
% xlim([omniTime(1), omniTime(end)]);
% yticklabels(yplain);
% grid on;    
% 
% legend_handles = [];
% legend_labels = {};
% for i = 1:7
%     if any(omni.method_used == i)
%         patch_handle = patch(NaN, NaN, method_colors{i, 2} , ...
%             'EdgeColor', 'none', 'DisplayName', method_colors{i, 4},'FaceAlpha', 0.2);
%         legend_handles = [legend_handles, patch_handle];
%         legend_labels = [legend_labels, method_colors{i, 4}];
%     end
% end
% legend(legend_handles, legend_labels, 'Location', 'bestoutside', 'Interpreter','latex', 'NumColumns', 1);
% saveas(fig1,strcat(figFolder,'controller_method.png'));    
% hold off;
% 
% fig2 = figure('name', 'Controller Commands', 'Position', [1, 1, 1920, 1080]);
% set(fig2, 'defaulttextinterpreter', 'latex');
% 
% 
% hold on;    
%  for interval = 1:(length(method_changes) - 1)
%     start_idx = method_changes(interval);
%     end_idx = method_changes(interval + 1) - 1;        
%     color_idx = omni.method_used(start_idx);
%     if color_idx >= 1 && color_idx <= 7
%         bg_color = method_colors{color_idx, 2};
%         if start_idx == 1
%             t_start = 0;
%         else
%             t_start = omniTime(start_idx);
%         end
% 
%         if (end_idx + 1) <= length(omniTime)
%             t_end = omniTime(end_idx + 1);
%         else
%             t_end = omniTime(end);
%         end            
%         fill([t_start, t_end, t_end, t_start], ...
%              [-300, -300, 300, 300], bg_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
%     end
%  end
% h1 = plot(rotTime, rotors.cmd_v0, 'LineWidth', lineSize, 'DisplayName', '$\omega_{1}$');
% h2 = plot(rotTime, rotors.cmd_v1, 'LineWidth', lineSize, 'DisplayName', '$\omega_{2}$');
% h3 = plot(rotTime, rotors.cmd_v2, 'LineWidth', lineSize, 'DisplayName', '$\omega_{3}$');
% h4 = plot(rotTime, rotors.cmd_v3, 'LineWidth', lineSize, 'DisplayName', '$\omega_{4}$');
% h5 = plot(rotTime, rotors.cmd_v4, 'LineWidth', lineSize, 'DisplayName', '$\omega_{5}$');
% h6 = plot(rotTime, rotors.cmd_v5, 'LineWidth', lineSize, 'DisplayName', '$\omega_{6}$');
% h7 = plot(rotTime, rotors.cmd_v6, 'LineWidth', lineSize, 'DisplayName', '$\omega_{7}$');
% h8 = plot(rotTime, rotors.cmd_v7, 'LineWidth', lineSize, 'DisplayName', '$\omega_{8}$');
% h9 = plot(rotTime, ones(1, numel(rotTime)) * 220, '--', 'LineWidth', lineSize,'DisplayName', '$\mathrm{upper\ limit}$');
% h10 = plot(rotTime, -ones(1, numel(rotTime)) * 220, '--', 'LineWidth', lineSize, 'DisplayName', '$-\mathrm{upper\ limit}$');
% h11 = plot(rotTime, ones(1, numel(rotTime)) * 30, '--', 'LineWidth', lineSize, 'DisplayName', '$\mathrm{lower\ limit}$');
% h12 = plot(rotTime, -ones(1, numel(rotTime)) * 30, '--', 'LineWidth', lineSize,  'DisplayName', '$-\mathrm{lower\ limit}$');
% y_lim = max([rotors.cmd_v0,rotors.cmd_v1,rotors.cmd_v2,rotors.cmd_v3,rotors.cmd_v4,rotors.cmd_v5,rotors.cmd_v6,rotors.cmd_v7]);
% xlabel('time [sec]');
% ylabel('propeller speed (Hz)');
% xlim([omniTime(1), omniTime(end)]);
% title('Commanded Propeller Speed', 'FontSize', 14, 'FontWeight', 'bold');   
% patch_handles = [];
% patch_labels = {};
% for i = 1:7
%     if any(omni.method_used == i)
%         patch_handle = patch(NaN, NaN, method_colors{i, 2} , ...
%             'EdgeColor', 'none', 'DisplayName', method_colors{i, 4},'FaceAlpha', 0.2);
%         patch_handles = [patch_handles, patch_handle];
%         patch_labels = [patch_labels, method_colors{i, 4}];
%     end
% end
% % all_handles = [h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, h11, h12, patch_handles];
% % all_labels = {'\omega_{1}', '\omega_{2}', '\omega_{3}', '\omega_{4}', ...
% %           '\omega_{5}', '\omega_{6}', '\omega_{7}', '\omega_{8}', ...
% %           'upper limit', '-upper limit', 'lower limit', '-lower limit', ...
% %           patch_labels{:}};
% % leg = legend(all_handles, all_labels, 'Location', 'bestoutside', ...
% %          'NumColumns', 1);
% % --- handles gruppi ---
% h_rot = [h1 h2 h3 h4 h5 h6 h7 h8];
% lab_rot = {'$\omega_{1}$','$\omega_{2}$','$\omega_{3}$','$\omega_{4}$', ...
%            '$\omega_{5}$','$\omega_{6}$','$\omega_{7}$','$\omega_{8}$'};
% 
% h_lim = [h9 h10 h11 h12];
% lab_lim = {'$\mathrm{upper\ limit}$','$-\mathrm{upper\ limit}$', ...
%            '$\mathrm{lower\ limit}$','$-\mathrm{lower\ limit}$'};
% 
% ax1 = gca;
% 
% % ========== 1) LEGENDA METODI SOTTO ==========
% leg_methods = legend(ax1, patch_handles, patch_labels, ...
%     'Location','southoutside', ...
%     'Orientation','horizontal', ...
%     'NumColumns', min(7, numel(patch_handles)), ...
%     'Interpreter','latex');
% set(leg_methods,'FontSize',legFontSize);
% 
% % ========== 2) LEGENDA ROTORI A DX ==========
% ax2 = axes('Position', ax1.Position, 'Color','none', ...
%            'XTick',[], 'YTick',[], 'Box','off', 'HitTest','off');
% axis(ax2,'off');
% 
% leg_rot = legend(ax2, h_rot, lab_rot, ...
%     'Location','eastoutside', ...
%     'Interpreter','latex');
% set(leg_rot,'FontSize',legFontSize);
% 
% % ========== 3) LEGENDA LIMITI A DX (sotto rotori) ==========
% ax3 = axes('Position', ax1.Position, 'Color','none', ...
%            'XTick',[], 'YTick',[], 'Box','off', 'HitTest','off');
% axis(ax3,'off');
% 
% leg_lim = legend(ax3, h_lim, lab_lim, ...
%     'Location','eastoutside', ...
%     'Interpreter','latex');
% set(leg_lim,'FontSize',legFontSize);
% 
% drawnow;                 
% pR = leg_rot.Position;
% pL = leg_lim.Position;
% 
% pL(1) = pR(1);                 % stessa colonna a destra
% pL(3) = pR(3);                 % stessa larghezza
% pL(2) = pR(2) - pL(4) - 0.02;  % sotto di un piccolo gap
% leg_lim.Position = pL;
% set(gca,'FontSize',14);
% grid on;
% hold off;
% saveas(fig2,strcat(figFolder,'rotor_speeds_method.png'));
% 
% 
% fig3 = figure('name', 'Controller Commands', 'Position', [1,1,1920,1080]);
% set(fig3, 'defaulttextinterpreter', 'latex');
% hold on;    
% y_best_matrix = [omni.ybest_1, omni.ybest_2, omni.ybest_3, omni.ybest_4, ...
%                  omni.ybest_5, omni.ybest_6, omni.ybest_7, omni.ybest_8]';
% y_labels = {'y1','y2','y3','y4','y5','y6','y7','y8'};
% nY = size(y_best_matrix, 1);    
% colors = [
%     0.00 0.45 0.74; 
%     0.85 0.33 0.10;
%     0.93 0.69 0.13;
%     0.49 0.18 0.56; 
%     0.47 0.67 0.19;
%     0.30 0.75 0.93;
%     1.00 0.00 1.00; 
%     0.64 0.08 0.18; 
% ];    
% for interval = 1:(length(method_changes) - 1)
%     start_idx = method_changes(interval);
%     end_idx = method_changes(interval + 1) - 1;        
%     color_idx = omni.method_used(start_idx);
%     if color_idx >= 1 && color_idx <= 7
%         bg_color = method_colors{color_idx, 2};
% 
%         if start_idx == 1
%             t_start = 0;
%         else
%             t_start = omniTime(start_idx);
%         end
%         if (end_idx + 1) <= length(omniTime)
%             t_end = omniTime(end_idx + 1);
%         else
%             t_end = omniTime(end);
%         end            
%         fill([t_start, t_end, t_end, t_start], ...
%              [-1, -1, nY+2, nY+2], bg_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
%              'HandleVisibility', 'off'); 
%     end
% end
% alpha = 0.5;
% step_separation = 1.0;
% base_offset = 0:step_separation:(nY-1)*step_separation;  
% offset_matrix = repmat(base_offset', 1, size(y_best_matrix, 2));
% y_shifted = (y_best_matrix * alpha) + offset_matrix;
% y_line_handles = gobjects(1, nY); 
% 
% for i = 1:nY
%     y_line_handles(i) = stairs(omniTime(1:length(y_shifted)), y_shifted(i,:), '--', ...
%                                'LineWidth', 1.5, 'Color', colors(i,:), ...
%                                'DisplayName', y_labels{i});
% end    
% method_patch_handles = gobjects(0);
% method_patch_labels = {};
%     if exist('method_colors', 'var')
%         for i = 1:7
%             if any(omni.method_used == i)
%                 patch_handle = patch(NaN, NaN, method_colors{i, 2}, ...
%                     'EdgeColor', 'none', 'FaceAlpha', 0.2, ...
%                     'DisplayName', method_colors{i, 4});
%                 method_patch_handles = [method_patch_handles, patch_handle];
%                 method_patch_labels = [method_patch_labels, yplain(i)];
%             end
%         end
%     end    
% yticks(base_offset + alpha/2);
% yticklabels(y_labels);
% ylim([-step_separation, max(base_offset) + alpha + step_separation*0.2]);
% xlabel('time (s)', 'FontSize', 12);
% ylabel('Y variables', 'FontSize', 12);
% xlim([omniTime(1), omniTime(end)]);    
% if ~isempty(method_patch_handles)
%     all_handles = [y_line_handles, method_patch_handles];
%     all_labels = [y_labels, method_patch_labels];        
%     empty_handle = plot(NaN, NaN, 'w', 'HandleVisibility', 'off');
%     all_handles = [all_handles, empty_handle];
%     all_labels = [all_labels, {' '}];
%     leg = legend(all_handles, all_labels, ...
%                  'Location', 'bestoutside',...
%                  'NumColumns', 1);
% 
% else
%     leg = legend(y_line_handles, y_labels, ...
%                  'Location', 'bestoutside', ...
%                  'NumColumns', 1);
% set(leg,'FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% 
% end
% title('Y variables activations', 'FontSize', 14, 'FontWeight', 'bold');
% xlim([omniTime(1), omniTime(end)]);
% grid on;
% hold off;  
% saveas(fig3,strcat(figFolder,'y_variables.png'));
% 
% fig4 = figure('name', 'Controller Usage Statistics', 'Position', [100, 100, 1200, 600]);    
% subplot(1, 2, 1);
% set(gca, 'Position', [0.07 0.20 0.4 0.70]);
% 
% counts = [omni.count_pinv(end), omni.count_miqp(end), ...
%           omni.count_miqp_relaxed(end), omni.count_qp(end), ...
%           omni.count_fallback_miqp(end), ...
%           omni.count_fallback_miqp_relaxed(end), omni.count_fallback_prev(end)];
% perc_counts = counts/n_samples*100;
% bar_colors = zeros(7, 3);
% for i = 1:7
%     bar_colors(i, :) = method_colors{i, 2};
% end    
% bar_handle = bar(1:7, perc_counts, 'FaceColor', 'flat','FaceAlpha', 0.2, 'CData', bar_colors);
% title('Percentage of uses per method \%', 'FontSize', 14, 'FontWeight', 'bold','Interpreter','latex');
% xticks(1:7);
% xticklabels(yplain);
% xtickangle(45);  
% for i = 1:7
%     if perc_counts(i) > 0
%         text(i, perc_counts(i), num2str(perc_counts(i), '%.3f'), ...
%             'HorizontalAlignment', 'center', ...
%             'VerticalAlignment', 'bottom', ...
%             'FontWeight', 'bold', 'FontSize', 9);
%     end
% end  
% 
% subplot(1, 2, 2);
% set(gca, 'Position', [0.57 0.20 0.4 0.70]);
% avg_times = [omni.avg_time_pinv(end), omni.avg_time_miqp(end), ...
%              omni.avg_time_miqp_relaxed(end), omni.avg_time_qp(end), ...
%              omni.avg_time_fallback_miqp(end), ...
%              omni.avg_time_fallback_miqp_relaxed(end), omni.avg_time_fallback_prev(end)];    
% bar(1:7, avg_times, 'FaceColor', 'flat', 'FaceAlpha', 0.2, 'CData', bar_colors);
% ylabel('time ($\mu$s)','FontSize', 14,'Interpreter','latex');
% title('Average execution time per method', 'FontSize', 14, 'FontWeight', 'bold','Interpreter','latex');
% xticks(1:7);
% xticklabels(yplain);
% xtickangle(45);
% for i = 1:7
%     if avg_times(i) > 0
%         text(i, avg_times(i), sprintf('%.1f', avg_times(i)), ...
%             'HorizontalAlignment', 'center', ...
%             'VerticalAlignment', 'bottom', ...
%             'FontWeight', 'bold', 'FontSize', 9);
%     end
% end
% annotation('textbox', [0.1, 0.02, 0.8, 0.05], 'String', ...
%     sprintf('Average time: %.1f $\\mu$s | Max time: %.1f $\\mu$s', ...
%     mean(omni.solve_time_us), max(omni.solve_time_us)), ...
%     'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
%     'FontSize', 14, 'FontWeight', 'bold','Interpreter','latex', 'EdgeColor', 'none', ...
%     'BackgroundColor', [0.95, 0.95, 0.95]);
% saveas(fig4,strcat(figFolder,'control_times.png'));
% hold off;


%%
fig2 = figure('name', 'Controller Commands', 'Position', [1, 1, 1920, 1080]);
set(fig2, 'defaulttextinterpreter', 'latex');
hold on;
h1 = plot(rotTime, rotors.cmd_v0, 'LineWidth', lineSize, 'DisplayName', '$\omega_{1}$');
h2 = plot(rotTime, rotors.cmd_v1, 'LineWidth', lineSize, 'DisplayName', '$\omega_{2}$');
h3 = plot(rotTime, rotors.cmd_v2, 'LineWidth', lineSize, 'DisplayName', '$\omega_{3}$');
h4 = plot(rotTime, rotors.cmd_v3, 'LineWidth', lineSize, 'DisplayName', '$\omega_{4}$');
h5 = plot(rotTime, rotors.cmd_v4, 'LineWidth', lineSize, 'DisplayName', '$\omega_{5}$');
h6 = plot(rotTime, rotors.cmd_v5, 'LineWidth', lineSize, 'DisplayName', '$\omega_{6}$');
h7 = plot(rotTime, rotors.cmd_v6, 'LineWidth', lineSize, 'DisplayName', '$\omega_{7}$');
h8 = plot(rotTime, rotors.cmd_v7, 'LineWidth', lineSize, 'DisplayName', '$\omega_{8}$');
h9 = plot(rotTime, ones(1, numel(rotTime)) * 220, '--', 'LineWidth', lineSize,'DisplayName', '$\mathrm{upper\ limit}$');
h10 = plot(rotTime, -ones(1, numel(rotTime)) * 220, '--', 'LineWidth', lineSize, 'DisplayName', '$-\mathrm{upper\ limit}$');
h11 = plot(rotTime, ones(1, numel(rotTime)) * 30, '--', 'LineWidth', lineSize, 'DisplayName', '$\mathrm{lower\ limit}$');
h12 = plot(rotTime, -ones(1, numel(rotTime)) * 30, '--', 'LineWidth', lineSize,  'DisplayName', '$-\mathrm{lower\ limit}$');
y_lim = max([rotors.cmd_v0,rotors.cmd_v1,rotors.cmd_v2,rotors.cmd_v3,rotors.cmd_v4,rotors.cmd_v5,rotors.cmd_v6,rotors.cmd_v7]);
xlabel('time [sec]');
ylabel('propeller speed (Hz)');
xlim([omniTime(1), omniTime(end)]);
title('Commanded Propeller Speed', 'FontSize', 14, 'FontWeight', 'bold');   
patch_handles = [];
patch_labels = {};

h_rot = [h1 h2 h3 h4 h5 h6 h7 h8];
lab_rot = {'$\omega_{1}$','$\omega_{2}$','$\omega_{3}$','$\omega_{4}$', ...
           '$\omega_{5}$','$\omega_{6}$','$\omega_{7}$','$\omega_{8}$'};

h_lim = [h9 h10 h11 h12];
lab_lim = {'$\mathrm{upper\ limit}$','$-\mathrm{upper\ limit}$', ...
           '$\mathrm{lower\ limit}$','$-\mathrm{lower\ limit}$'};

ax1 = gca;

% ========== 2) LEGENDA ROTORI A DX ==========
ax2 = axes('Position', ax1.Position, 'Color','none', ...
           'XTick',[], 'YTick',[], 'Box','off', 'HitTest','off');
axis(ax2,'off');

leg_rot = legend(ax2, h_rot, lab_rot, ...
    'Location','eastoutside', ...
    'Interpreter','latex');
set(leg_rot,'FontSize',legFontSize);

% ========== 3) LEGENDA LIMITI A DX (sotto rotori) ==========
ax3 = axes('Position', ax1.Position, 'Color','none', ...
           'XTick',[], 'YTick',[], 'Box','off', 'HitTest','off');
axis(ax3,'off');

leg_lim = legend(ax3, h_lim, lab_lim, ...
    'Location','eastoutside', ...
    'Interpreter','latex');
set(leg_lim,'FontSize',legFontSize);

drawnow;                 
pR = leg_rot.Position;
pL = leg_lim.Position;

pL(1) = pR(1);                 % stessa colonna a destra
pL(3) = pR(3);                 % stessa larghezza
pL(2) = pR(2) - pL(4) - 0.02;  % sotto di un piccolo gap
leg_lim.Position = pL;
set(gca,'FontSize',14);
grid on;
hold off;
saveas(fig2,strcat(figFolder,'rotor_speeds_method.png'));
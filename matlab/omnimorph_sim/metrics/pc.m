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

    run('../omnimorph_params')
end

%% Extract data
extractLog(strcat(logFolder, 'man.log'),'man');
extractLog(strcat(logFolder, 'phyn.log'),'phynt');
extractLog(strcat(logFolder, 'stats.log'),'pom');
extractLog(strcat(logFolder, 'rot.log'),'rotors');
extractLog(strcat(logFolder, 'omni.log'),'omni');
extractLog(strcat(logFolder, 'opt.log'),'opti');

% FT sensor data
extractFTSensorData(strcat(logFolder,'FT'),'FT') 
FT.ts = posixtime(FT.ts); 

% Extract times
globalStart = min([pom.ts(1), phynt.ts(1), man.ts(1), rotors.ts(1), omni.ts(1) FT.ts(1)]);
pomTime   = pom.ts   - globalStart;
phyntTime = phynt.ts - globalStart;
manTime = man.ts - globalStart;
rotTime = rotors.ts - globalStart;
omniTime = omni.ts - globalStart;
ftTime = FT.ts - globalStart;

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

% Remove NaN values
validPhynt = ~isnan(phynt.efx) & ~isnan(phynt.efy) & ~isnan(phynt.efz); % Take out NAN values
phyntValTime = phyntTime(validPhynt);

validRot = ~isnan(rotors.ts) & ~isnan(rotors.meas_v0) & ~isnan(rotors.meas_v1) & ~isnan(rotors.meas_v2) & ~isnan(rotors.meas_v3) & ~isnan(rotors.meas_v4) & ~isnan(rotors.meas_v5) & ~isnan(rotors.meas_v6) & ~isnan(rotors.meas_v7); % Take out NAN values
rotMeasTime = rotTime(validRot);

% Propeller saturations for plotting
maxPropellerMeas = ones(1,numel(rotMeasTime))*config.uavParams.maxPropSpeed;
minPropellerMeas = ones(1,numel(rotMeasTime))*config.uavParams.minPropSpeed;

%% Plot and check surface frame orientation
i=1;

while true
    if strcmp(opti.name(i),'Rigid_Body_3')
        R = eul2rotm([opti.yaw(i), opti.pitch(i), opti.roll(i)],"ZYX");
        break
    end
    i=i+1;
end

% Define axes
origin = [0;0;0];
worldAxes = eye(3);      % World frame axes
surfAxes = R * eye(3);   % Rotated body frame axes

% Plot
figure(1); hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(35,25);

% Plot world frame (dashed)
quiver3(origin(1), origin(2), origin(3), ...
        worldAxes(1,1), worldAxes(2,1), worldAxes(3,1), 'r--', 'LineWidth', 1.5, 'MaxHeadSize',0.5);
quiver3(origin(1), origin(2), origin(3), ...
        worldAxes(1,2), worldAxes(2,2), worldAxes(3,2), 'g--', 'LineWidth', 1.5, 'MaxHeadSize',0.5);
quiver3(origin(1), origin(2), origin(3), ...
        worldAxes(1,3), worldAxes(2,3), worldAxes(3,3), 'b--', 'LineWidth', 1.5, 'MaxHeadSize',0.5);

% Plot body frame (solid)
quiver3(origin(1), origin(2), origin(3), ...
        surfAxes(1,1), surfAxes(2,1), surfAxes(3,1), 'r', 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver3(origin(1), origin(2), origin(3), ...
        surfAxes(1,2), surfAxes(2,2), surfAxes(3,2), 'g', 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver3(origin(1), origin(2), origin(3), ...
        surfAxes(1,3), surfAxes(2,3), surfAxes(3,3), 'b', 'LineWidth', 2, 'MaxHeadSize',0.5);

legend('X_w','Y_w','Z_w','X_b','Y_b','Z_b');

%% FT sensor data processing
% Force at the sensor frame such that it coincides with the surface frame
sensForce = [-FT.Fx; -FT.Fy; FT.Fz]; %b z-outside surface  
%sensForce = [FT.Fx; -FT.Fy; -FT.Fz]; % z-inside surface   

% Transform world frame force to surface frame
bias = 1.2;
estForce = R' * [phynt.efx(validPhynt)';phynt.efy(validPhynt)';(phynt.efz(validPhynt)-bias)'];

% CoF for PLA - rubber/leather
th = [27, 26, 26.4, 27, 26, 26.5, 26, 25.8, 27, 25.4];
mu = tan(deg2rad(th));
meanMu = mean(mu);

% Friction cone
meanFrictCone = meanMu*abs(sensForce(3,:));
tanForces = sqrt(sensForce(1,:).^2 + sensForce(2,:).^2);

%% Plot measured vs estimated force in surface frame
% Visualization parameters
lineSize = 2;
legFontSize = 20;

fig = figure('name','Interaction wrench estimation', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
ax1 = subplot(2,1,1);
hold on
grid on
plot(phyntValTime,estForce(1,:),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{x}}','$$'));
plot(phyntValTime,estForce(2,:),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{y}}','$$'));
plot(phyntValTime,-estForce(3,:),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{z}}','$$'));
plot(ftTime,-sensForce(1,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{x}}','$$'));
plot(ftTime,-sensForce(2,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{y}}','$$'));
plot(ftTime,sensForce(3,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{z}}','$$'));

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Force [N]','FontSize',legFontSize)
title('Estimated and measured force on end-effector Vs time','FontSize',legFontSize)

ax2 = subplot(2,1,2);
hold on
grid on
plot(ftTime,meanFrictCone,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\mu_s f_z','$$'));
plot(ftTime,tanForces,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\sqrt{f_x^2 + f_y^2}','$$'));

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
title('Friction cone and tangential forces Vs time','FontSize',legFontSize)

linkaxes([ax1,ax2],'x')

%saveas(fig,strcat(figFolder,'PC_force.png'));

%% Point contact plot - All states
% visualization parameters
folders = split(baseFolder,'/');
flightName = folders(end-1);

figSize = [100,100,2100,600];
lineSize = 1.5;
legFontSize = 12;
labFontSize = 15;
tickSize = 10;
gridSize = 1.5;
xMin = 8;
xMax = 2;
iconWidth = 15;

fig = figure('Position',figSize); % [left bottom width height]
tiled = tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact'); % Create a 4x2 grid
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
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
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
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[m/s]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Velocity",'FontSize',labFontSize)

ax3 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,rotors.meas_v0(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotMeasTime,rotors.meas_v1(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotMeasTime,rotors.meas_v2(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotMeasTime,rotors.meas_v3(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotMeasTime,rotors.meas_v4(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotMeasTime,rotors.meas_v5(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotMeasTime,rotors.meas_v6(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotMeasTime,rotors.meas_v7(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Propeller speeds",'FontSize',labFontSize)

ax4 = nexttile; % Move to the next tile
hold on
grid on
plot(phyntValTime(1:end-1000),estForce(1,1:end-1000),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{x}}','$$'));
plot(phyntValTime(1:end-1000),estForce(2,1:end-1000),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{y}}','$$'));
plot(phyntValTime(1:end-1000),estForce(3,1:end-1000),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{z}}','$$'));
plot(ftTime,-sensForce(1,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{x}}','$$'));
plot(ftTime,-sensForce(2,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{y}}','$$'));
plot(ftTime,-sensForce(3,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{z}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[N]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Estimated and measured force",'FontSize',labFontSize)

ax5 = nexttile; % Move to the next tile
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
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("roll-pitch-yaw",'FontSize',labFontSize)

ax6 = nexttile; % Move to the next tile
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
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg/s]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 

ax7 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,(rotors.cmd_v0(validRot) - rotors.meas_v0(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
plot(rotMeasTime,(rotors.cmd_v1(validRot) - rotors.meas_v1(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
plot(rotMeasTime,(rotors.cmd_v2(validRot) - rotors.meas_v2(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
plot(rotMeasTime,(rotors.cmd_v3(validRot) - rotors.meas_v3(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
plot(rotMeasTime,(rotors.cmd_v4(validRot) - rotors.meas_v4(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
plot(rotMeasTime,(rotors.cmd_v5(validRot) - rotors.meas_v5(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
plot(rotMeasTime,(rotors.cmd_v6(validRot) - rotors.meas_v6(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
plot(rotMeasTime,(rotors.cmd_v7(validRot) - rotors.meas_v7(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Propeller speed error','FontSize',labFontSize)

ax8 = nexttile; % Move to the next tile
hold on
grid on
plot(ftTime,meanFrictCone,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\mu_s \\ ^Sf_z','$$'));
plot(ftTime,tanForces,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\sqrt{^Sf_x^2 + \\ ^Sf_y^2}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[N]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Friction cone and tangential force','FontSize',labFontSize)


%% Point contact plot - postion, input and measured force 
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
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[m]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Position",'FontSize',labFontSize)

ax2 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,rotors.meas_v0(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotMeasTime,rotors.meas_v1(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotMeasTime,rotors.meas_v2(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotMeasTime,rotors.meas_v3(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotMeasTime,rotors.meas_v4(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotMeasTime,rotors.meas_v5(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotMeasTime,rotors.meas_v6(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotMeasTime,rotors.meas_v7(validRot),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(rotMeasTime,maxPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')
plot(rotMeasTime,minPropellerMeas,'--','LineWidth',lineSize,'HandleVisibility','off')

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Propeller speeds",'FontSize',labFontSize)

ax3 = nexttile; % Move to the next tile
hold on
grid on
plot(phyntValTime(1:end-1000),estForce(1,1:end-1000),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{x}}','$$'));
plot(phyntValTime(1:end-1000),estForce(2,1:end-1000),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{y}}','$$'));
plot(phyntValTime(1:end-1000),estForce(3,1:end-1000),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','^S\hat{f}_{E_{z}}','$$'));
plot(ftTime,-sensForce(1,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{x}}','$$'));
plot(ftTime,-sensForce(2,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{y}}','$$'));
plot(ftTime,-sensForce(3,:),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^Sf_{E_{z}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'XTickLabel', [],'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[N]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("Estimated and measured force",'FontSize',labFontSize)

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
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[deg]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title("roll-pitch-yaw",'FontSize',labFontSize)

ax5 = nexttile; % Move to the next tile
hold on
grid on
plot(rotMeasTime,(rotors.cmd_v0(validRot) - rotors.meas_v0(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{1}}','$$'));
plot(rotMeasTime,(rotors.cmd_v1(validRot) - rotors.meas_v1(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{2}}','$$'));
plot(rotMeasTime,(rotors.cmd_v2(validRot) - rotors.meas_v2(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{3}}','$$'));
plot(rotMeasTime,(rotors.cmd_v3(validRot) - rotors.meas_v3(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{4}}','$$'));
plot(rotMeasTime,(rotors.cmd_v4(validRot) - rotors.meas_v4(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{5}}','$$'));
plot(rotMeasTime,(rotors.cmd_v5(validRot) - rotors.meas_v5(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{6}}','$$'));
plot(rotMeasTime,(rotors.cmd_v6(validRot) - rotors.meas_v6(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{7}}','$$'));
plot(rotMeasTime,(rotors.cmd_v7(validRot) - rotors.meas_v7(validRot)),'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{\omega_{8}}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[Hz]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Propeller speed error','FontSize',labFontSize)
xlabel('time [sec]','FontSize',labFontSize)

ax6 = nexttile; % Move to the next tile
hold on
grid on
plot(ftTime,meanFrictCone,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\mu_s \\ ^Sf_z','$$'));
plot(ftTime,tanForces,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\sqrt{^Sf_x^2 + \\ ^Sf_y^2}','$$'));

leg = legend('show');
set(leg,'Interpreter','latex','FontSize',legFontSize,"Location","northeast","Orientation","horizontal",'IconColumnWidth',iconWidth);
set(gca,'FontSize',tickSize,'GridLineStyle', '-','GridAlpha', 0.3);
ylabel('[N]','FontSize',labFontSize)
xlim([xMin, globalEnd+xMax]) 
yl = ylim;                       
ylim([yl(1)-0.1*range(yl), yl(2)+0.2*range(yl)]);  
title('Friction cone and tangential force','FontSize',labFontSize)


%f = gcf;
%exportgraphics(gcf,strcat(figFolder,'states.pdf'),'ContentType','vector')


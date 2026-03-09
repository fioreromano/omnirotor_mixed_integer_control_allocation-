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

    addpath('./lib/')
    addpath('./lib/omnimorph-helper/')
    addpath('./lib/math-helper/')
    addpath('./sim/')
    addpath('./plot/')

    run('./omnimorph_params')
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
omniTime = omni.ts - globalStart ;

globalEnd = max([pomTime(end), phyntTime(end), manTime(end), rotTime(end), omniTime(end)]);

%% Extract end-effector position and velocity from Pom
N = numel(pom.ts);
pomEndEffector = pom;
pomEndEffector.x = zeros(N,1);
pomEndEffector.y = zeros(N,1);
pomEndEffector.z = zeros(N,1);

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

% End-effector error
diffX = pomEndEffector.x - phyntX_interp;
diffY = pomEndEffector.y - phyntY_interp;
diffZ = pomEndEffector.z - phyntZ_interp;

%% visualization parameters
lineSize = 2;
legFontSize = 20;


%% Plot UAV body position
fig2 = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');

subplot(2,1,1)
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
title('UAV body (CoM) Position vs Time','FontSize',legFontSize)

subplot(2,1,2)
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
title('UAV body Orientation vs Time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'body_pos.png'));

%% Plot UAV body error
fig2 = figure('name','UAV Error', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(omniTime,omni.e_x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
plot(omniTime,omni.e_y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
plot(omniTime,omni.e_z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('pos [meters]','FontSize',legFontSize)
title('UAV body (CoM) Position Error vs Time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(omniTime,rad2deg(omni.e_rx), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\phi}','$$')); 
plot(omniTime,rad2deg(omni.e_ry), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\theta}','$$')); 
plot(omniTime,rad2deg(omni.e_rz), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\psi}','$$')); 

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize)
title('UAV body Orientation Error vs Time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'body_pos_err.png'));

%% Plot UAV body velocities
fig2 = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(pomTime,pom.vx,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}_{B}','$$'));
plot(pomTime,pom.vy,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}_{B}','$$'));
plot(pomTime,pom.vz,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}_{B}','$$'));

plot(omniTime,omni.vxd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{x}^{d}_{B}','$$'));
plot(omniTime,omni.vyd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{y}^{d}_{B}','$$'));
plot(omniTime,omni.vzd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{z}^{d}_{B}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('vel [meters/sec]','FontSize',legFontSize)
title('UAV body Translation Vel vs Time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(pomTime,rad2deg(pom.wx),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}_{B}','$$'));
plot(pomTime,rad2deg(pom.wy),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}_{B}','$$'));
plot(pomTime,rad2deg(pom.wz),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}_{B}','$$'));

plot(omniTime,rad2deg(omni.wxd),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\phi}^{d}_{B}','$$'));
plot(omniTime,rad2deg(omni.wyd),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\theta}^{d}_{B}','$$'));
plot(omniTime,rad2deg(omni.wzd),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\dot{\psi}^{d}_{B}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('omega [degrees/sec]','FontSize',legFontSize)
title('UAV body Angular Vel vs Time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'body_vel.png'));


%% Plot UAV body accelerations
fig2 = figure('name','UAV State', 'Position', [1 1 1920 1080/2]);% get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');

hold on
grid on
plot(pomTime,pom.ax,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\ddot{x}_{B}','$$'));
plot(pomTime,pom.ay,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\ddot{y}_{B}','$$'));
plot(pomTime,pom.az,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\ddot{z}_{B}','$$'));

plot(omniTime,omni.axd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\ddot{x}^{d}_{B}','$$'));
plot(omniTime,omni.ayd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\ddot{y}^{d}_{B}','$$'));
plot(omniTime,omni.azd,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\ddot{z}^{d}_{B}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('acc [meters/sec*sec]','FontSize',legFontSize)
title('UAV body Translation Acc vs Time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'body_acc.png'));

%% Plot UAV end-effector position
% fig = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
% set(fig,'defaulttextinterpreter','latex');
% subplot(2,1,1)
% hold on
% grid on
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
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('pos [meters]','FontSize',legFontSize)
% title('UAV end-effector position vs time','FontSize',legFontSize)
% 
% subplot(2,1,2)
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
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize)
% title('UAV end-effector orientation vs time','FontSize',legFontSize)
% 
% saveas(fig,strcat(figFolder,'ee_pos.png'));

%% Plot UAV end-effector error
% fig = figure('name','UAV Error', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
% set(fig,'defaulttextinterpreter','latex');
% subplot(2,1,1)
% hold on
% grid on
% plot(pomTime,diffX,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{x}','$$'));
% plot(pomTime,diffY,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{y}','$$'));
% plot(pomTime,diffZ,'LineWidth',lineSize,'DisplayName' ,strcat('$$','e_{z}','$$'));
% 
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('pos [meters]','FontSize',legFontSize)
% title('UAV end-effector position error vs time','FontSize',legFontSize)
% 
% % Orientation error assumes R^B_E=eye(3)
% subplot(2,1,2)
% hold on
% grid on
% plot(omniTime,rad2deg(omni.e_rx), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\phi}','$$')); 
% plot(omniTime,rad2deg(omni.e_ry), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\theta}','$$')); 
% plot(omniTime,rad2deg(omni.e_rz), 'LineWidth', lineSize, 'DisplayName', strcat('$$','e_{\psi}','$$')); 
% 
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize)
% title('UAV end-effector orientation error vs time','FontSize',legFontSize)
% 
% saveas(fig,strcat(figFolder,'ee_pos_err.png'));

%% Plot UAV end-effector velocities
% fig = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
% set(fig,'defaulttextinterpreter','latex');
% subplot(2,1,1)
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
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('vel [meters/sec]','FontSize',legFontSize)
% title('UAV end-effector Translation Vel vs Time','FontSize',legFontSize)
% 
% subplot(2,1,2)
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
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('omega [degrees/sec]','FontSize',legFontSize)
% title('UAV end-effector angular vel vs time','FontSize',legFontSize)
% 
% saveas(fig,strcat(figFolder,'ee_vel.png'));

%% Plotting propeller commanded and measured speeds
valid = ~isnan(rotors.ts) & ~isnan(rotors.meas_v0) & ~isnan(rotors.meas_v1) & ~isnan(rotors.meas_v2) & ~isnan(rotors.meas_v3) & ~isnan(rotors.meas_v4) & ~isnan(rotors.meas_v5) & ~isnan(rotors.meas_v6) & ~isnan(rotors.meas_v7); % Take out NAN values
rotMeasTime = rotors.ts(valid) - globalStart;

config.uavParams.maxPropSpeed = 300;
config.uavParams.minPropSpeed = -300;
maxPropeller = ones(1,numel(rotors.ts))*config.uavParams.maxPropSpeed;
minPropeller = ones(1,numel(rotors.ts))*config.uavParams.minPropSpeed;

fig2 = figure('name','Controller Commands', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
ax1 = subplot(3,1,1);
hold on
grid on
plot(rotTime,rotors.cmd_v0,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotTime,rotors.cmd_v1,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotTime,rotors.cmd_v2,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotTime,rotors.cmd_v3,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotTime,rotors.cmd_v4,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotTime,rotors.cmd_v5,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotTime,rotors.cmd_v6,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotTime,rotors.cmd_v7,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(rotTime,maxPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{upper\ limit}','$$'))
plot(rotTime,minPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{lower\ limit}','$$'))
%plot(rotTime,ones(1,numel(rotors.ts))*30,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{lower\ limit}','$$'))
%plot(rotTime,-ones(1,numel(rotors.ts))*30,'--','LineWidth',lineSize,'DisplayName', strcat('$$','-\mathrm{lower\ limit}','$$'))

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-8);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize-3)
ylabel('propeller speed [Hz] ','FontSize',legFontSize-3)
title('Commanded Propeller Speeds vs Time','FontSize',legFontSize-3)

ax2 = subplot(3,1,2);
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

plot(rotMeasTime,ones(1,numel(rotMeasTime))*config.uavParams.maxPropSpeed,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{upper\ limit}','$$'))
plot(rotMeasTime,ones(1,numel(rotMeasTime))*config.uavParams.minPropSpeed,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{lower\ limit}','$$'))
%plot(rotMeasTime,ones(1,numel(rotMeasTime))*30,'--','LineWidth',lineSize, 'DisplayName', strcat('$$','\mathrm{lower\ limit}','$$'))
%plot(rotMeasTime,-ones(1,numel(rotMeasTime))*30,'--','LineWidth',lineSize, 'DisplayName', strcat('$$','-\mathrm{lower\ limit}','$$'))


leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-8);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize-3)
ylabel('propeller speed [Hz] ','FontSize',legFontSize-3)
title('Measured Propeller Speeds vs Time','FontSize',legFontSize-3)

ax3 = subplot(3,1,3);
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

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-8);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize-3)
ylabel('propeller speed [Hz] ','FontSize',legFontSize-3)
title('Propeller Speeds Error vs Time','FontSize',legFontSize-3)

linkaxes([ax1, ax2, ax3], 'x');

saveas(fig2,strcat(figFolder,'props.png'));

%% Propeller thrusts
meas_thrust1 = config.uavParams.c_f*sign(rotors.meas_v0(valid)).*(rotors.meas_v0(valid).^2);
meas_thrust2 = config.uavParams.c_f*sign(rotors.meas_v1(valid)).*(rotors.meas_v1(valid).^2);
meas_thrust3 = config.uavParams.c_f*sign(rotors.meas_v2(valid)).*(rotors.meas_v2(valid).^2);
meas_thrust4 = config.uavParams.c_f*sign(rotors.meas_v3(valid)).*(rotors.meas_v3(valid).^2);
meas_thrust5 = config.uavParams.c_f*sign(rotors.meas_v4(valid)).*(rotors.meas_v4(valid).^2);
meas_thrust6 = config.uavParams.c_f*sign(rotors.meas_v5(valid)).*(rotors.meas_v5(valid).^2);
meas_thrust7 = config.uavParams.c_f*sign(rotors.meas_v6(valid)).*(rotors.meas_v6(valid).^2);
meas_thrust8 = config.uavParams.c_f*sign(rotors.meas_v7(valid)).*(rotors.meas_v7(valid).^2);

maxPropellerThrustMeas = ones(1,numel(rotMeasTime))*(config.uavParams.c_f*sign(config.uavParams.maxPropSpeed)*(config.uavParams.maxPropSpeed.^2));
minPropellerThrustMeas = ones(1,numel(rotMeasTime))*(config.uavParams.c_f*sign(config.uavParams.minPropSpeed)*(config.uavParams.minPropSpeed.^2));

fig2 = figure('name','Controller Commands', 'Position', [1,1,1920,1080/2]);%, get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
hold on
grid on
plot(rotMeasTime,meas_thrust1,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{1}}','$$'));
plot(rotMeasTime,meas_thrust2,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{2}}','$$'));
plot(rotMeasTime,meas_thrust3,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{3}}','$$'));
plot(rotMeasTime,meas_thrust4,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{4}}','$$'));
plot(rotMeasTime,meas_thrust5,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{5}}','$$'));
plot(rotMeasTime,meas_thrust6,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{6}}','$$'));
plot(rotMeasTime,meas_thrust7,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{7}}','$$'));
plot(rotMeasTime,meas_thrust8,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{P_{8}}','$$'));

plot(rotMeasTime,maxPropellerThrustMeas,'--','LineWidth',lineSize,'DisplayName', strcat('$$','upper limit','$$'))
plot(rotMeasTime,minPropellerThrustMeas,'--','LineWidth',lineSize,'DisplayName', strcat('$$','lower limit','$$'))

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('propeller thrust [N] ','FontSize',legFontSize)
title('Estimated Propeller Thrusts vs Time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'props_thrust.png'));

%% Commanded Propellers Speed
fig2 = figure('name','Controller Commands', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
hold on
grid on
plot(rotTime,rotors.cmd_v0,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(rotTime,rotors.cmd_v1,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(rotTime,rotors.cmd_v2,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(rotTime,rotors.cmd_v3,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(rotTime,rotors.cmd_v4,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(rotTime,rotors.cmd_v5,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(rotTime,rotors.cmd_v6,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(rotTime,rotors.cmd_v7,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(rotTime,maxPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{upper\ limit}','$$'))
plot(rotTime,minPropeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{lower\ limit}','$$'))
% plot(rotTime,ones(1,numel(rotors.ts))*30,'--','LineWidth',lineSize,'DisplayName', strcat('$$','\mathrm{lower\ limit}','$$'))
% plot(rotTime,-ones(1,numel(rotors.ts))*30,'--','LineWidth',lineSize,'DisplayName', strcat('$$','-\mathrm{lower\ limit}','$$'))

leg = legend('show','Location','NorthEast');
set(leg,'Interpreter','latex','FontSize',legFontSize-8);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize-3);
ylabel('propeller speed [Hz] ','FontSize',legFontSize-3);
title('Commanded Propeller Speeds vs Time','FontSize',legFontSize-3);
saveas(fig2,strcat(figFolder,'props_speed.png'));

%% Body Wrench estimation
fig2 = figure('name','Body Wrench estimation', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(phyntTime(1:end-1000),phynt.efx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{x}}','$$'));
plot(phyntTime(1:end-1000),phynt.efy(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{y}}','$$'));
plot(phyntTime(1:end-1000),phynt.efz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{B_{z}}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Force [N]','FontSize',legFontSize)
title('Estimated force on body vs time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(phyntTime(1:end-1000),phynt.etx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{x}}','$$'));
plot(phyntTime(1:end-1000),phynt.ety(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{y}}','$$'));
plot(phyntTime(1:end-1000),phynt.etz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{B_{z}}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Moment [Nm]','FontSize',legFontSize)
title('Estimated moment on body vs time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'wrench.png'));

%% EE Wrench estimation
% fig = figure('name','EE Wrench estimation', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
% set(fig,'defaulttextinterpreter','latex');
% subplot(2,1,1)
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.efx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{E_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.efy(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{E_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.efz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{f}_{E_{z}}','$$'));
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('Force [N]','FontSize',legFontSize)
% title('Estimated force on end-effector vs Time','FontSize',legFontSize)
% 
% subplot(2,1,2)
% hold on
% grid on
% plot(phyntTime(1:end-1000),phynt.eetx(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{E_{x}}','$$'));
% plot(phyntTime(1:end-1000),phynt.eety(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{E_{y}}','$$'));
% plot(phyntTime(1:end-1000),phynt.eetz(1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^W\hat{m}_{E_{z}}','$$'));
% leg = legend('show','Location','NorthEastOutside');
% set(leg,'Interpreter','latex','FontSize',legFontSize);
% set(gca,'FontSize',legFontSize-10);
% xlabel('time [sec]','FontSize',legFontSize)
% ylabel('Moment [Nm]','FontSize',legFontSize)
% title('Estimated moment on end-effector vs Time','FontSize',legFontSize)
% 
% saveas(fig,strcat(figFolder,'ee_wrench.png'));
% 
% %% Body Wrench estimation
N = numel(phynt.ts);

for i = 1:N
    % Extract roll, pitch, yaw and position
    eul = [phynt.yaw(i), phynt.pitch(i), phynt.roll(i)];  % [Z Y X]

    % Compute rotation matrix (assumes desired approximately equal to
    % actual orientation)
    R = eul2rotm(eul, 'ZYX');

    % Add to the world-frame position
    bodForce(:,i) = R' * [phynt.efx(i); phynt.efy(i); phynt.efz(i)];
    bodTorque(:,i) = R' * [phynt.etx(i); phynt.ety(i); phynt.etz(i)];
end

fig2 = figure('name','Body wrench estimation', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig2,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(phyntTime(1:end-1000),bodForce(1,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{x}}','$$'));
plot(phyntTime(1:end-1000),bodForce(2,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{y}}','$$'));
plot(phyntTime(1:end-1000),bodForce(3,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{z}}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Force [N]','FontSize',legFontSize)
title('Estimated force on body vs Time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(phyntTime(1:end-1000),bodTorque(1,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{x}}','$$'));
plot(phyntTime(1:end-1000),bodTorque(2,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{y}}','$$'));
plot(phyntTime(1:end-1000),bodTorque(3,1:end-1000),'LineWidth',lineSize,'DisplayName' ,strcat('$$','^B\hat{f}_{B_{z}}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Force [N]','FontSize',legFontSize)
title('Estimated Torque on Body vs Time','FontSize',legFontSize)

saveas(fig2,strcat(figFolder,'bod_wrench.png'));

%% Plot Optimization Results
% if controllerChoice == 4
% 
%     method_colors = {
%         1, [0.90, 0.40, 0.40], 'Rosso',  'pinv';  
%         2, [0.35, 0.70, 0.90], 'Azzurro',  'miqp';                
%         3, [0.45, 0.85, 0.45], 'Verde',     'miqp_{relaxed}';      
%         4, [0.95, 0.80, 0.20], 'Giallo',    'qp';                 
%         5, [0.70, 0.40, 0.90], 'Viola','fallback_{miqp}';      
%         6, [0.30, 0.90, 0.90], 'Ciano','fallback_{miqp-relaxed}';
%         7, [0.95, 0.60, 0.20], 'Arancio',   'fallback_{pinv}';     
%     };    
%     n_samples = length(omni.method_used);    
%     fig1 = figure('name', 'Controller Methods Timeline', 'Position', [100, 100, 1400, 500]);
%     hold on;
%     if n_samples > 1
%         method_changes = [1; find(diff(omni.method_used) ~= 0) + 1; n_samples + 1];
%     else
%         method_changes = [1; n_samples + 1];
%     end
%     for interval = 1:(length(method_changes) - 1)
%         start_idx = method_changes(interval);
%         end_idx = method_changes(interval + 1) - 1;        
%         color_idx = omni.method_used(start_idx);
%         if color_idx >= 1 && color_idx <= 7
%             bg_color = method_colors{color_idx, 2};
%             if start_idx == 1
%                 t_start = 0;
%             else
%                 t_start = omniTime(start_idx);
%             end
% 
%             if (end_idx + 1) <= length(omniTime)
%                 t_end = omniTime(end_idx + 1);
%             else
%                 t_end = omniTime(end);
%             end            
%             fill([t_start, t_end, t_end, t_start], ...
%                  [0, 0, method_colors{color_idx, 1}, method_colors{color_idx, 1}], bg_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
%         end
%     end  
%     stairs(omniTime(1:min(end, n_samples)), omni.method_used, 'k--','LineWidth', 0.1);
%     xlabel('time (s)');
%     ylabel('method');
%     title('Controller Methods Timeline', 'FontSize', 12, 'FontWeight', 'bold');
%     ylim([0.5, 7.5]);
%     yticks(1:7);
%     xlim([omniTime(1), omniTime(end)]);
%     yticklabels({method_colors{:,4}});
%     grid on;    
% 
%     legend_handles = [];
%     legend_labels = {};
%     for i = 1:7
%         if any(omni.method_used == i)
%             patch_handle = patch(NaN, NaN, method_colors{i, 2} , ...
%                 'EdgeColor', 'none', 'DisplayName', method_colors{i, 4},'FaceAlpha', 0.2);
%             legend_handles = [legend_handles, patch_handle];
%             legend_labels = [legend_labels, method_colors{i, 4}];
%         end
%     end
%     legend(legend_handles, legend_labels, 'Location', 'bestoutside', 'NumColumns', 1);
%     saveas(fig1,strcat(figFolder,'controller_method.png'));    
%     hold off;
% 
% 
%     fig2 = figure('name', 'Controller Commands', 'Position', [1, 1, 1920, 1080]);
%     set(fig2, 'defaulttextinterpreter', 'latex');
% 
% 
%     hold on;    
%      for interval = 1:(length(method_changes) - 1)
%         start_idx = method_changes(interval);
%         end_idx = method_changes(interval + 1) - 1;        
%         color_idx = omni.method_used(start_idx);
%         if color_idx >= 1 && color_idx <= 7
%             bg_color = method_colors{color_idx, 2};
%             if start_idx == 1
%                 t_start = 0;
%             else
%                 t_start = omniTime(start_idx);
%             end
% 
%             if (end_idx + 1) <= length(omniTime)
%                 t_end = omniTime(end_idx + 1);
%             else
%                 t_end = omniTime(end);
%             end            
%             fill([t_start, t_end, t_end, t_start], ...
%                  [-300, -300, 300, 300], bg_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
%         end
%      end
%     h1 = plot(rotTime, rotors.cmd_v0, 'LineWidth', lineSize, 'DisplayName', '$\omega_{1}$');
%     h2 = plot(rotTime, rotors.cmd_v1, 'LineWidth', lineSize, 'DisplayName', '$\omega_{2}$');
%     h3 = plot(rotTime, rotors.cmd_v2, 'LineWidth', lineSize, 'DisplayName', '$\omega_{3}$');
%     h4 = plot(rotTime, rotors.cmd_v3, 'LineWidth', lineSize, 'DisplayName', '$\omega_{4}$');
%     h5 = plot(rotTime, rotors.cmd_v4, 'LineWidth', lineSize, 'DisplayName', '$\omega_{5}$');
%     h6 = plot(rotTime, rotors.cmd_v5, 'LineWidth', lineSize, 'DisplayName', '$\omega_{6}$');
%     h7 = plot(rotTime, rotors.cmd_v6, 'LineWidth', lineSize, 'DisplayName', '$\omega_{7}$');
%     h8 = plot(rotTime, rotors.cmd_v7, 'LineWidth', lineSize, 'DisplayName', '$\omega_{8}$');
%     h9 = plot(rotTime, maxPropeller, '--', 'LineWidth', lineSize, 'Color', [1 0 0], 'DisplayName', '$\mathrm{upper\ limit}$');
%     h10 = plot(rotTime, minPropeller, '--', 'LineWidth', lineSize, 'Color', [1 0 0], 'DisplayName', '$-\mathrm{upper\ limit}$');
%     h11 = plot(rotTime, ones(1, numel(rotTime)) * 30, '--', 'LineWidth', lineSize, 'Color', [0 0 1], 'DisplayName', '$\mathrm{lower\ limit}$');
%     h12 = plot(rotTime, -ones(1, numel(rotTime)) * 30, '--', 'LineWidth', lineSize, 'Color', [0 0 1], 'DisplayName', '$-\mathrm{lower\ limit}$');
%     y_lim = max([rotors.cmd_v0,rotors.cmd_v1,rotors.cmd_v2,rotors.cmd_v3,rotors.cmd_v4,rotors.cmd_v5,rotors.cmd_v6,rotors.cmd_v7]);
%     xlabel('time (s)');
%     ylabel('propeller speed (Hz)');
%     xlim([omniTime(1), omniTime(end)]);
%     title('Commanded Propeller Speed', 'FontSize', 14, 'FontWeight', 'bold');   
%     patch_handles = [];
%     patch_labels = {};
%     for i = 1:7
%         if any(omni.method_used == i)
%             patch_handle = patch(NaN, NaN, method_colors{i, 2} , ...
%                 'EdgeColor', 'none', 'DisplayName', method_colors{i, 4},'FaceAlpha', 0.2);
%             patch_handles = [patch_handles, patch_handle];
%             patch_labels = [patch_labels, method_colors{i, 4}];
%         end
%     end
%     all_handles = [h1, h2, h3, h4, h5, h6, h7, h8, h9, h10, h11, h12, patch_handles];
%     all_labels = {'\omega_{1}', '\omega_{2}', '\omega_{3}', '\omega_{4}', ...
%               '\omega_{5}', '\omega_{6}', '\omega_{7}', '\omega_{8}', ...
%               'upper limit', '-upper limit', 'lower limit', '-lower limit', ...
%               patch_labels{:}};
%     leg = legend(all_handles, all_labels, 'Location', 'bestoutside', ...
%              'NumColumns', 1);
%     set(leg,'FontSize',legFontSize-8);
%     set(gca,'FontSize',legFontSize-10);
%     grid on;
%     hold off;
%     saveas(fig2,strcat(figFolder,'rotor_speeds_method.png'));
% 
% 
%     fig3 = figure('name', 'Controller Commands', 'Position', [1,1,1920,1080]);
%     set(fig3, 'defaulttextinterpreter', 'latex');
%     hold on;    
%     y_best_matrix = [omni.ybest_1, omni.ybest_2, omni.ybest_3, omni.ybest_4, ...
%                      omni.ybest_5, omni.ybest_6, omni.ybest_7, omni.ybest_8]';
%     y_labels = {'y1','y2','y3','y4','y5','y6','y7','y8'};
%     nY = size(y_best_matrix, 1);    
%     colors = [
%         0.00 0.45 0.74; 
%         0.85 0.33 0.10;
%         0.93 0.69 0.13;
%         0.49 0.18 0.56; 
%         0.47 0.67 0.19;
%         0.30 0.75 0.93;
%         1.00 0.00 1.00; 
%         0.64 0.08 0.18; 
%     ];    
%     for interval = 1:(length(method_changes) - 1)
%         start_idx = method_changes(interval);
%         end_idx = method_changes(interval + 1) - 1;        
%         color_idx = omni.method_used(start_idx);
%         if color_idx >= 1 && color_idx <= 7
%             bg_color = method_colors{color_idx, 2};
% 
%             if start_idx == 1
%                 t_start = 0;
%             else
%                 t_start = omniTime(start_idx);
%             end
%             if (end_idx + 1) <= length(omniTime)
%                 t_end = omniTime(end_idx + 1);
%             else
%                 t_end = omniTime(end);
%             end            
%             fill([t_start, t_end, t_end, t_start], ...
%                  [-1, -1, nY+2, nY+2], bg_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
%                  'HandleVisibility', 'off'); 
%         end
%     end
%     alpha = 0.5;
%     step_separation = 1.0;
%     base_offset = 0:step_separation:(nY-1)*step_separation;  
%     offset_matrix = repmat(base_offset', 1, size(y_best_matrix, 2));
%     y_shifted = (y_best_matrix * alpha) + offset_matrix;
%     y_line_handles = gobjects(1, nY); 
% 
%     for i = 1:nY
%         y_line_handles(i) = stairs(omniTime(1:length(y_shifted)), y_shifted(i,:), '--', ...
%                                    'LineWidth', 1.5, 'Color', colors(i,:), ...
%                                    'DisplayName', y_labels{i});
%     end    
%     method_patch_handles = gobjects(0);
%     method_patch_labels = {};
%         if exist('method_colors', 'var')
%             for i = 1:7
%                 if any(omni.method_used == i)
%                     patch_handle = patch(NaN, NaN, method_colors{i, 2}, ...
%                         'EdgeColor', 'none', 'FaceAlpha', 0.2, ...
%                         'DisplayName', method_colors{i, 4});
%                     method_patch_handles = [method_patch_handles, patch_handle];
%                     method_patch_labels = [method_patch_labels, method_colors{i, 4}];
%                 end
%             end
%         end    
%     yticks(base_offset + alpha/2);
%     yticklabels(y_labels);
%     ylim([-step_separation, max(base_offset) + alpha + step_separation*0.2]);
%     xlabel('time (s)', 'FontSize', 12);
%     ylabel('Y variables', 'FontSize', 12);
%     xlim([omniTime(1), omniTime(end)]);    
%     if ~isempty(method_patch_handles)
%         all_handles = [y_line_handles, method_patch_handles];
%         all_labels = [y_labels, method_patch_labels];        
%         empty_handle = plot(NaN, NaN, 'w', 'HandleVisibility', 'off');
%         all_handles = [all_handles, empty_handle];
%         all_labels = [all_labels, {' '}];
%         leg = legend(all_handles, all_labels, ...
%                      'Location', 'bestoutside',...
%                      'NumColumns', 1);
% 
%     else
%         leg = legend(y_line_handles, y_labels, ...
%                      'Location', 'bestoutside', ...
%                      'NumColumns', 1);
%     set(leg,'FontSize',legFontSize-8);
%     set(gca,'FontSize',legFontSize-10);
% 
%     end
%     title('Y variables activations', 'FontSize', 14, 'FontWeight', 'bold');
%     xlim([omniTime(1), omniTime(end)]);
%     grid on;
%     hold off;  
%     saveas(fig3,strcat(figFolder,'y_variables.png'));
% 
%     fig4 = figure('name', 'Controller Usage Statistics', 'Position', [100, 100, 1200, 600]);    
%     subplot(1, 2, 1);
%     set(gca, 'Position', [0.07 0.20 0.4 0.70]);
% 
%     counts = [omni.count_pinv(end), omni.count_miqp(end), ...
%               omni.count_miqp_relaxed(end), omni.count_qp(end), ...
%               omni.count_fallback_miqp(end), ...
%               omni.count_fallback_miqp_relaxed(end), omni.count_fallback_prev(end)];
% 
%     bar_colors = zeros(7, 3);
%     for i = 1:7
%         bar_colors(i, :) = method_colors{i, 2};
%     end    
%     bar_handle = bar(1:7, counts, 'FaceColor', 'flat','FaceAlpha', 0.2, 'CData', bar_colors);
%     ylabel('total count','Interpreter','latex');
%     title('Number of uses per method ', 'FontSize', 14, 'FontWeight', 'bold','Interpreter','latex');
%     set(gca, 'XTick', 1:7, 'XTickLabel', {method_colors{:,4}});
%     xtickangle(45);
%     grid on;    
%     for i = 1:7
%         if counts(i) > 0
%             text(i, counts(i), num2str(counts(i)), ...
%                 'HorizontalAlignment', 'center', ...
%                 'VerticalAlignment', 'bottom', ...
%                 'FontWeight', 'bold', 'FontSize', 7);
%         end
%     end  
% 
%     subplot(1, 2, 2);
%     set(gca, 'Position', [0.57 0.20 0.4 0.70]);    
%     avg_times = [omni.avg_time_pinv(end), omni.avg_time_miqp(end), ...
%                  omni.avg_time_miqp_relaxed(end), omni.avg_time_qp(end), ...
%                  omni.avg_time_fallback_miqp(end), ...
%                  omni.avg_time_fallback_miqp_relaxed(end), omni.avg_time_fallback_prev(end)];    
%     bar(1:7, avg_times, 'FaceColor', 'flat', 'FaceAlpha', 0.2, 'CData', bar_colors);
%     ylabel('time ($\mu$s)','Interpreter','latex');
%     title('Average execution time per method', 'FontSize', 14, 'FontWeight', 'bold','Interpreter','latex');
%     set(gca, 'XTick', 1:7, 'XTickLabel', {method_colors{:,4}});
%     xtickangle(45);
%     grid on;
%     for i = 1:7
%         if avg_times(i) > 0
%             text(i, avg_times(i), sprintf('%.1f', avg_times(i)), ...
%                 'HorizontalAlignment', 'center', ...
%                 'VerticalAlignment', 'bottom', ...
%                 'FontWeight', 'bold', 'FontSize', 7);
%         end
%     end
%     annotation('textbox', [0.1, 0.02, 0.8, 0.05], 'String', ...
%         sprintf('Total count: %d | Average time: %.1f us | Max time: %.1f us', ...
%         n_samples, mean(omni.solve_time_us), max(omni.solve_time_us)), ...
%         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
%         'FontSize', 12, 'FontWeight', 'bold','Interpreter','latex', 'EdgeColor', 'none', ...
%         'BackgroundColor', [0.95, 0.95, 0.95]);
%     saveas(fig4,strcat(figFolder,'control_times.png'));
%     hold off;
% end

%%
valid_cmd = ~isnan(rotors.ts) & ~isnan(rotors.cmd_v0) & ~isnan(rotors.cmd_v1) & ...
            ~isnan(rotors.cmd_v2) & ~isnan(rotors.cmd_v3) & ~isnan(rotors.cmd_v4) & ...
            ~isnan(rotors.cmd_v5) & ~isnan(rotors.cmd_v6) & ~isnan(rotors.cmd_v7);

t_cmd = rotors.ts(valid_cmd) - globalStart;

U = [rotors.cmd_v0(valid_cmd) ...
        rotors.cmd_v1(valid_cmd) ...
        rotors.cmd_v2(valid_cmd) ...
        rotors.cmd_v3(valid_cmd) ...
        rotors.cmd_v4(valid_cmd) ...
        rotors.cmd_v5(valid_cmd) ...
        rotors.cmd_v6(valid_cmd) ...
        rotors.cmd_v7(valid_cmd)].';
U = abs(U);
Pcmd = config.uavParams.c_t*(U.^(3/2));
Ptot_t = sum(Pcmd,1);
% Compute per-rotor energy
Ei = trapz(t_cmd, Pcmd, 2);
Etot = sum(Ei);
avgY = mean(Etot);               % average effort


fig1 = figure('Color','w'); 
subplot(2,1,1)
hold on;
for i = 1:8
    plot(t_cmd, Pcmd(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Rotor %d', i));
end
xlabel('time [s]','FontSize', 12, 'Interpreter', 'latex');
ylabel('$c_{\tau}u_{\omega_i}^{3/2}$','FontSize', 12, 'Interpreter', 'latex');
title('Per-rotor power vs time','FontSize', 16, 'FontWeight', 'bold', 'Interpreter', 'latex');
legend('Location','bestoutside');
xlim([0, t_cmd(end)]); grid on;
subplot(2,1,2)
plot(t_cmd, Ptot_t, 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Total Power'));
xlabel('time [s]','FontSize', 12, 'Interpreter', 'latex');
ylabel('$\sum c_{\tau}u_{\omega_i}^{3/2} $','FontSize', 12, 'Interpreter', 'latex');
title('Total power vs time','FontSize', 16, 'FontWeight', 'bold', 'Interpreter', 'latex');
xlim([0, t_cmd(end)]); grid on; box off;
legend('Location','bestoutside');
saveas(fig1,strcat(figFolder,'per_rotor_power.png'));

fig2 = figure;
b = bar(Ei, 'FaceColor', [0.2 0.6 0.9], 'LineWidth', 0.5);
grid on; box off;
xlabel('Rotor $i$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$\int c_{\tau}u_{\omega_i}^{3/2} \, dt$', 'FontSize', 12, 'Interpreter', 'latex');
title('Per-rotor effort', 'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'latex');
ylim([0 max(Ei)*1.1]);
text(0.98, 0.95, sprintf('Total effort: %.3f', Etot), ...
        'Units', 'normalized', ...           
        'HorizontalAlignment', 'right', ...  
        'VerticalAlignment', 'top', ...     
        'FontSize', 12, ...
        'Interpreter', 'latex', ...
        'BackgroundColor', 'w', ...          
        'EdgeColor', 'k');
saveas(fig2,strcat(figFolder,'per_rotor_effort.png'));
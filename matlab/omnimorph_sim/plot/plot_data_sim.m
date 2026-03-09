%% visualization parameters
lineSize =2;
legFontSize=20;

datalen = size(tout,1);
time = tout(:,1);

%% UAV end-effector position
x = zeros(1,datalen);
y = zeros(1,datalen);
z = zeros(1,datalen);
xr = zeros(1,datalen);
yr = zeros(1,datalen);
zr = zeros(1,datalen);
xc = zeros(1,datalen);
yc = zeros(1,datalen);
zc = zeros(1,datalen);

x(1,:) = p_e_w(1,1,:);
y(1,:) = p_e_w(2,1,:);
z(1,:) = p_e_w(3,1,:);
xr(1,:) = p_e_ref(1,1,:);
yr(1,:) = p_e_ref(2,1,:);
zr(1,:) = p_e_ref(3,1,:);
xc(1,:) = p_e_comp(1,1,:);
yc(1,:) = p_e_comp(2,1,:);
zc(1,:) = p_e_comp(3,1,:);

roll = zeros(1,datalen);
pitch = zeros(1,datalen);
yaw = zeros(1,datalen);
roll(1,:) = rpy_e(3,1,:);
pitch(1,:) = rpy_e(2,1,:);
yaw(1,:) = rpy_e(1,1,:);

roll_r = zeros(1,datalen);
pitch_r = zeros(1,datalen);
yaw_r = zeros(1,datalen);
roll_r(1,:) = rpy_e_ref(3,1,:);
pitch_r(1,:) = rpy_e_ref(2,1,:);
yaw_r(1,:) = rpy_e_ref(1,1,:);

roll_c = zeros(1,datalen);
pitch_c = zeros(1,datalen);
yaw_c = zeros(1,datalen);
roll_c(1,:) = rpy_e_comp(3,1,:);
pitch_c(1,:) = rpy_e_comp(2,1,:);
yaw_c(1,:) = rpy_e_comp(1,1,:);

fig = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(time,x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{e}','$$'));
plot(time,y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{e}','$$'));
plot(time,z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{e}','$$'));

plot(time,xr,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{des}','$$'));
plot(time,yr,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{des}','$$'));
plot(time,zr,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{des}','$$'));

plot(time,xc,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{comp}','$$'));
plot(time,yc,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{comp}','$$'));
plot(time,zc,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{comp}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('pos [meters]','FontSize',legFontSize)
title('UAV end-effector position Vs time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(time,rad2deg(roll),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
plot(time,rad2deg(pitch),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
plot(time,rad2deg(yaw),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));

plot(time,rad2deg(roll_r),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi_{des}','$$'));
plot(time,rad2deg(pitch_r),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta_{des}','$$'));
plot(time,rad2deg(yaw_r),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi_{des}','$$'));

plot(time,rad2deg(roll_c),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi_{comp}','$$'));
plot(time,rad2deg(pitch_c),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta_{comp}','$$'));
plot(time,rad2deg(yaw_c),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi_{comp}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('roll-pitch-yaw [degrees]','FontSize',legFontSize)
title('UAV end-effector orientation vs time','FontSize',legFontSize)

saveas(fig,strcat(figFolder,'ee_pos_',sessionName,'.png'));

%% UAV end effector velocity 
xd = zeros(1,datalen);
yd = zeros(1,datalen);
zd = zeros(1,datalen);
xd_r = zeros(1,datalen);
yd_r = zeros(1,datalen);
zd_r = zeros(1,datalen);
xd_c = zeros(1,datalen);
yd_c = zeros(1,datalen);
zd_c = zeros(1,datalen);

xd(1,:) = p_e_dt(1,1,:);
yd(1,:) = p_e_dt(2,1,:);
zd(1,:) = p_e_dt(3,1,:);
xd_r(1,:) = p_e_dt_ref(1,1,:);
yd_r(1,:) = p_e_dt_ref(2,1,:);
zd_r(1,:) = p_e_dt_ref(3,1,:);
xd_c(1,:) = p_e_dt_comp(1,1,:);
yd_c(1,:) = p_e_dt_comp(2,1,:);
zd_c(1,:) = p_e_dt_comp(3,1,:);

omx = zeros(1,datalen);
omy = zeros(1,datalen);
omz = zeros(1,datalen);
omx(1,:) = omega_e(1,1,:);
omy(1,:) = omega_e(2,1,:);
omz(1,:) = omega_e(3,1,:);

omx_r = zeros(1,datalen);
omy_r = zeros(1,datalen);
omz_r = zeros(1,datalen);
omx_r(1,:) = omega_e_ref(1,1,:);
omy_r(1,:) = omega_e_ref(2,1,:);
omz_r(1,:) = omega_e_ref(3,1,:);

omx_c = zeros(1,datalen);
omy_c = zeros(1,datalen);
omz_c = zeros(1,datalen);
omx_c(1,:) = omega_e_comp(1,1,:);
omy_c(1,:) = omega_e_comp(2,1,:);
omz_c(1,:) = omega_e_comp(3,1,:);

fig = figure('name','UAV State', 'Position', [1 1 1920 1080]);% get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(time,xd,'LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{e}','$$'));
plot(time,yd,'LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{e}','$$'));
plot(time,zd,'LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{e}','$$'));

plot(time,xd_r,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{des}','$$'));
plot(time,yd_r,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{des}','$$'));
plot(time,zd_r,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{des}','$$'));

plot(time,xd_c,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','x_{comp}','$$'));
plot(time,yd_c,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','y_{comp}','$$'));
plot(time,zd_c,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','z_{comp}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('vel [meters/sec]','FontSize',legFontSize)
title('UAV end-effector translation vel Vs time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(time,rad2deg(omx),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi','$$'));
plot(time,rad2deg(omy),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta','$$'));
plot(time,rad2deg(omz),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi','$$'));

plot(time,rad2deg(omx_r),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi_{des}','$$'));
plot(time,rad2deg(omy_r),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta_{des}','$$'));
plot(time,rad2deg(omz_r),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi_{des}','$$'));

plot(time,rad2deg(omx_c),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\phi_{comp}','$$'));
plot(time,rad2deg(omy_c),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\theta_{comp}','$$'));
plot(time,rad2deg(omz_c),'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\psi_{comp}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('omega [degrees/sec]','FontSize',legFontSize)
title('UAV end-effector angular vel vs time','FontSize',legFontSize)

saveas(fig,strcat(figFolder,'ee_vel_',sessionName,'.png'));

%% Plotting propeller commanded and measured speeds
max_propeller = ones(1,datalen)*config.uavParams.maxPropSpeed;
min_propeller = ones(1,datalen)*config.uavParams.minPropSpeed;

fig = figure('name','Controller Commands', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(time,cmnd_prop_speeds(:,1),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(time,cmnd_prop_speeds(:,2),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(time,cmnd_prop_speeds(:,3),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(time,cmnd_prop_speeds(:,4),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(time,cmnd_prop_speeds(:,5),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(time,cmnd_prop_speeds(:,6),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(time,cmnd_prop_speeds(:,7),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(time,cmnd_prop_speeds(:,8),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('propeller speed [Hz] ','FontSize',legFontSize)
title('Comanded Propeller speeds Vs time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(time,meas_prop_speeds(:,1),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{1}','$$'));
plot(time,meas_prop_speeds(:,2),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{2}','$$'));
plot(time,meas_prop_speeds(:,3),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{3}','$$'));
plot(time,meas_prop_speeds(:,4),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{4}','$$'));
plot(time,meas_prop_speeds(:,5),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{5}','$$'));
plot(time,meas_prop_speeds(:,6),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{6}','$$'));
plot(time,meas_prop_speeds(:,7),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{7}','$$'));
plot(time,meas_prop_speeds(:,8),'LineWidth',lineSize,'DisplayName' ,strcat('$$','\omega_{8}','$$'));

plot(time,max_propeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','upper limit','$$'))
plot(time,min_propeller,'--','LineWidth',lineSize,'DisplayName', strcat('$$','lower limit','$$'))

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('propeller speed [Hz] ','FontSize',legFontSize)
title('Measured Propeller speeds Vs time','FontSize',legFontSize)

saveas(fig,strcat(figFolder,'props_',sessionName,'.png'));

%% Wrench estimation
f_b_est_x = zeros(1,datalen);
f_b_est_y = zeros(1,datalen);
f_b_est_z = zeros(1,datalen);
f_b_est_x(1,:) = w_b_est(1,1,:);
f_b_est_y(1,:) = w_b_est(2,1,:);
f_b_est_z(1,:) = w_b_est(3,1,:);

f_e_est_x = zeros(1,datalen);
f_e_est_y = zeros(1,datalen);
f_e_est_z = zeros(1,datalen);
f_e_est_x(1,:) = w_e_est(1,1,:);
f_e_est_y(1,:) = w_e_est(2,1,:);
f_e_est_z(1,:) = w_e_est(3,1,:);

m_b_est_x = zeros(1,datalen);
m_b_est_y = zeros(1,datalen);
m_b_est_z = zeros(1,datalen);
m_b_est_x(1,:) = w_b_est(4,1,:);
m_b_est_y(1,:) = w_b_est(5,1,:);
m_b_est_z(1,:) = w_b_est(6,1,:);

m_e_est_x = zeros(1,datalen);
m_e_est_y = zeros(1,datalen);
m_e_est_z = zeros(1,datalen);
m_e_est_x(1,:) = w_e_est(4,1,:);
m_e_est_y(1,:) = w_e_est(5,1,:);
m_e_est_z(1,:) = w_e_est(6,1,:);

fig = figure('name','Wrench estimation', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(time,f_b_est_x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{B_{x}}','$$'));
plot(time,f_b_est_y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{B_{y}}','$$'));
plot(time,f_b_est_z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{B_{z}}','$$'));
plot(time,f_e_est_x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{E_{x}}','$$'));
plot(time,f_e_est_y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{E_{y}}','$$'));
plot(time,f_e_est_z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{f}_{E_{z}}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Force [N]','FontSize',legFontSize)
title('Estimated force on body and end-effector Vs time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(time,m_b_est_x,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{m}_{B_{x}}','$$'));
plot(time,m_b_est_y,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{m}_{B_{y}}','$$'));
plot(time,m_b_est_z,'--','LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{m}_{B_{z}}','$$'));
plot(time,m_e_est_x,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{m}_{E_{x}}','$$'));
plot(time,m_e_est_y,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{m}_{E_{y}}','$$'));
plot(time,m_e_est_z,'LineWidth',lineSize,'DisplayName' ,strcat('$$','\hat{m}_{E_{z}}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('Moment [Nm]','FontSize',legFontSize)
title('Estimated moment on body and end-effector Vs time','FontSize',legFontSize)

saveas(fig,strcat(figFolder,'wrench_',sessionName,'.png'));

%% Propeller thrusts
cmnd_thrust1 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,1)).*(cmnd_prop_speeds(:,1).^2);
cmnd_thrust2 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,2)).*(cmnd_prop_speeds(:,2).^2);
cmnd_thrust3 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,3)).*(cmnd_prop_speeds(:,3).^2);
cmnd_thrust4 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,4)).*(cmnd_prop_speeds(:,4).^2);
cmnd_thrust5 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,5)).*(cmnd_prop_speeds(:,5).^2);
cmnd_thrust6 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,6)).*(cmnd_prop_speeds(:,6).^2);
cmnd_thrust7 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,7)).*(cmnd_prop_speeds(:,7).^2);
cmnd_thrust8 = config.uavParams.c_f*sign(cmnd_prop_speeds(:,8)).*(cmnd_prop_speeds(:,8).^2);

meas_thrust1 = config.uavParams.c_f*sign(meas_prop_speeds(:,1)).*(meas_prop_speeds(:,1).^2);
meas_thrust2 = config.uavParams.c_f*sign(meas_prop_speeds(:,2)).*(meas_prop_speeds(:,2).^2);
meas_thrust3 = config.uavParams.c_f*sign(meas_prop_speeds(:,3)).*(meas_prop_speeds(:,3).^2);
meas_thrust4 = config.uavParams.c_f*sign(meas_prop_speeds(:,4)).*(meas_prop_speeds(:,4).^2);
meas_thrust5 = config.uavParams.c_f*sign(meas_prop_speeds(:,5)).*(meas_prop_speeds(:,5).^2);
meas_thrust6 = config.uavParams.c_f*sign(meas_prop_speeds(:,6)).*(meas_prop_speeds(:,6).^2);
meas_thrust7 = config.uavParams.c_f*sign(meas_prop_speeds(:,7)).*(meas_prop_speeds(:,7).^2);
meas_thrust8 = config.uavParams.c_f*sign(meas_prop_speeds(:,8)).*(meas_prop_speeds(:,8).^2);

fig = figure('name','Controller Commands', 'Position', [1,1,1920,1080]);%, get(0, 'Screensize'));
set(fig,'defaulttextinterpreter','latex');
subplot(2,1,1)
hold on
grid on
plot(time,cmnd_thrust1,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{1}','$$'));
plot(time,cmnd_thrust2,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{2}','$$'));
plot(time,cmnd_thrust3,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{3}','$$'));
plot(time,cmnd_thrust4,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{4}','$$'));
plot(time,cmnd_thrust5,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{5}','$$'));
plot(time,cmnd_thrust6,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{6}','$$'));
plot(time,cmnd_thrust7,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{7}','$$'));
plot(time,cmnd_thrust8,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{8}','$$'));
leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('propeller speed [N] ','FontSize',legFontSize)
title('Comanded Propeller thrusts Vs time','FontSize',legFontSize)

subplot(2,1,2)
hold on
grid on
plot(time,meas_thrust1,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{1}','$$'));
plot(time,meas_thrust2,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{2}','$$'));
plot(time,meas_thrust3,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{3}','$$'));
plot(time,meas_thrust4,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{4}','$$'));
plot(time,meas_thrust5,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{5}','$$'));
plot(time,meas_thrust6,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{6}','$$'));
plot(time,meas_thrust7,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{7}','$$'));
plot(time,meas_thrust8,'LineWidth',lineSize,'DisplayName' ,strcat('$$','f_{8}','$$'));

leg = legend('show','Location','NorthEastOutside');
set(leg,'Interpreter','latex','FontSize',legFontSize);
set(gca,'FontSize',legFontSize-10);
xlabel('time [sec]','FontSize',legFontSize)
ylabel('propeller thrust [N] ','FontSize',legFontSize)
title('Measured Propeller thrusts Vs time','FontSize',legFontSize)

saveas(fig,strcat(figFolder,'thrust_',sessionName,'.png'));

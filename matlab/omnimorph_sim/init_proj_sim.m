%close all
clear
clc

%% Adding the matlab folders to matlab path
user = 'fiore';
currentFolder = pwd;

addpath(strcat(pwd,'/lib/'))
addpath(strcat(pwd,'/lib/omnimorph-helper/'))
addpath(strcat(pwd,'/lib/math-helper/'))
addpath(strcat(pwd,'/sim/'))
addpath(strcat(pwd,'/plot/'))

addpath(strcat('/home/',user,'/openrobots/lib/matlab/simulink/genomix/'))
addpath(strcat('/home/',user,'/openrobots/lib/matlab/simulink/'))
addpath(strcat('/home/',user,'/openrobots/lib/matlab/'))

%% Running parameters file 
run(strcat(pwd,'/omnimorph_params.m'))

%% Connecting to devices and load telekyb components
client = genomix.client('localhost');
pause(0.1)
rotorcraft = client.load('rotorcraft');
pause(0.1)
dynamixel = client.load('dynamixel');
pause(0.1)
optitrack = client.load('optitrack');
pause(0.1)
pom = client.load('pom');
pause(0.1)
rotorcraft.connect('/tmp/pty-omnimorph',500000)
pause(0.1)
dynamixel.connect('/tmp/pty-dynamixel')
pause(0.1)
optitrack.connect('localhost','1509','','0')
pause(0.1)

%% Set parameters and connect telekyb components
alpha_angle = config.uavParams.propTilt;
dynamixel.set_position({alpha_angle,alpha_angle,alpha_angle,alpha_angle,alpha_angle,alpha_angle,alpha_angle,alpha_angle})
pause(0.2)

rotorcraft.set_sensor_rate(500, 1, 50, 1)
pause(0.1)
 
imu_filter.gyro = {20, 20, 20};
imu_filter.acc = {5, 5, 5};
imu_filter.mag = {0,0,0};

rotorcraft.set_imu_filter(imu_filter.gyro,imu_filter.acc, imu_filter.mag);
pause(0.1)

pom.set_mag_field(23.8e-06, -0.4e-06, -39.8e-06)

pom.connect_port('measure/mag', 'rotorcraft/mag')
pom.add_measurement('mag')
pause(0.1)

pom.connect_port('measure/imu', 'rotorcraft/imu')
pom.add_measurement('imu');
pause(0.1)

pom.connect_port('measure/mocap', 'optitrack/bodies/omni_morph');
pom.add_measurement('mocap');
pause(0.1)

pom.set_history_length(0.3);
pause(0.1)

pom.set_prediction_model('::pom::constant_acceleration')
pause(0.1)

rotorcraft.start();

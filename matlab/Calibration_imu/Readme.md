# Introduction

  This folder contains all the necessary files to perform the IMU calibration of a flying 
  platform, hereafter called robot for brevity.

# File structure

* *imu_calibration.m*

  `MATLAB` script to be launched for calibrating the robot's IMU.

## Folder launch

* *launch_imu_calibration.sh*

  File `.sh` to be launched in the robot's pc or on your machine (change accordingly value of 
  variable `host_name` in `imu_calibration.m`). 
  It allows to run `GenoM` modules needed for the `Mikrokopter's IMU` calibration.

* *kill_all_pocolibs.sh*

  File `.sh` to be launched after having calibrated the IMU to kill all 
  `GenoM` processes of the modules during calibration.

# Steps

## In localhost, i.e. on your pc
- Give permissions to scripts `.sh`:
   $ chmod + x launch/*
- Run `launch_imu_calibration.sh`
   $ ./launch/launch_imu_calibration.sh
- Open `MATLAB` script `imu_calibration.m` and edit its Config section accordingly
- Run that `MATLAB` script
- Go back to terminal
- Follow instructions
- Once calibration ended successfully, kill all `GenoM` processes
  $ ./kill_all_pocolibs.sh

## On a robot's pc
- Login with your username to the robot's pc
- Copy through `scp` files `.sh`
  $ scp <your_username>@<your_pc>:<path_to_file>/imu_calibration.sh <path_in_robot_pc>/.
- Give permissions to them
  $ chmod +x <file>.sh
  es: $ chmod +x launch_imu_calibration.sh
- Run `launch_imu_calibration.sh`
  $ ./launch_imu_calibration.sh
- On your pc, open `MATLAB` script `imu_calibration.m`  and edit `Config section`
- Run the `MATLAB` script
- Go back to drone's terminal
- Follow instructions
- Once calibration ended successfully, kill all `GenoM` processes
  $ ./kill_all_pocolibs.sh


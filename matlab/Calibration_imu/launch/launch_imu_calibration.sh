#!/bin/bash

### Description:
#
# Launch Genomixd and Mikrokopter, which are the only Genom3-modules needed to
# calibrate Mikrokopter's IMU.

h2 init
sleep 2

genomixd &
rotorcraft-pocolibs -f 

#!/bin/bash

### Description:
#
# Kill most commom Genom3-modules.

pkill genomixd &
pkill mikrokopter-pocolibs &
pkill pom-pocolibs &
pkill optitrack-pocolibs &
pkill joystick-pocolibs &
pkill nhfc-pocolibs &
pkill mrsim-pocolibs &
h2 end 

#!/bin/bash

set -e

source /opt/ros/kinetic/setup.bash

cd /tmp/source/ros
catkin_make clean
catkin_make
source ./devel/setup.bash

cd /tmp/source
./run-pylint.sh

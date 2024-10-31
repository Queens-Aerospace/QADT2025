#!/bin/bash
if [ "$(basename "$PWD")" = "QADT2025" ]
then
echo "DO NOT RUN THIS IN THE BASE DIRECTORY!!!!"
echo "Run this in your workspace for it to do anything"
else
source /opt/ros/humble/setup.bash
source install/local_setup.bash
fi
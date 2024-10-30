#!/bin/bash
if [basename "$PWD" -eq "QADT2025"]
then
echo "DO NOT RUN THIS IN THE BASE DIRECTORY!!!!"
echo "Make a new folder. 'mkdir example'"
echo "And cd into that. 'cd example'"
echo "THEN you can run this"
else
mkdir src
cd src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
fi
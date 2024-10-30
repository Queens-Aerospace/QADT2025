#!/bin/bash
gnome-terminal --tab --title="Gazebo" -- bash -c \
'cd PX5;make px4_sitl gz_x500; exec bash'
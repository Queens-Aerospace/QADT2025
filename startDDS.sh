#!/bin/bash
gnome-terminal --tab --title="Micro DDS" -- bash -c \
'cd; cd Micro-XRCE-DDS-Agent;MicroXRCEAgent udp4 -p 8888; exec bash'
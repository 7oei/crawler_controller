#!/bin/bash

gnome-terminal -- bash -c "rosbag record -a --split --duration=1800 -O /home/adachi/`date +%Y%m%d%H%M`.bag; exec bash"
sleep 4
# source ~/.bashrc
roslaunch crawler_controller move_only.launch
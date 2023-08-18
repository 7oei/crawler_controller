#!/bin/bash

sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1
sudo chmod 777 /dev/ttyACM2
sudo chmod 777 /dev/ttyACM3
sudo chmod 777 /dev/ttyACM4
sudo chmod 777 /dev/ttyACM5
sudo chmod 777 /dev/ttyACM6
sleep 3

echo 'Running str2str command...'
gnome-terminal -- bash -c "/home/adachi/RTKLIB/app/consapp/str2str/gcc/str2str -in ntrip://ntrip1.bizstation.jp:2101/10D85AA2 -out serial://serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:230400; exec bash"
sleep 3

sudo ifconfig enp3s0f1 192.168.1.77
sudo route add 192.168.1.201 enp3s0f1
sleep 3

gnome-terminal -- bash -c "rosbag record -a --split --duration=1800 -O /home/adachi/`date +%Y%m%d%H%M`.bag; exec bash"
sleep 3

# ROSパッケージを起動
echo 'Launching ROS package...'
# source ~/.bashrc
roslaunch crawler_controller map_logging.launch

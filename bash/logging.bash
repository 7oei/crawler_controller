#!/bin/bash

# USBデバイスに対してパーミッションを設定
# echo 'Setting permissions for USB devices...'





sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1
sudo chmod 777 /dev/ttyACM2
sudo chmod 777 /dev/ttyACM3
sudo chmod 777 /dev/ttyACM4
sudo chmod 777 /dev/ttyACM5
sudo chmod 777 /dev/ttyACM6
sleep 5

echo 'Running str2str command...'
gnome-terminal -- bash -c "/home/adachi/RTKLIB/app/consapp/str2str/gcc/str2str -in ntrip://ntrip1.bizstation.jp:2101/10D85AA2 -out serial://serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:230400; exec bash"

sudo ifconfig enp3s0f1 192.168.1.77
sudo route add 192.168.1.201 enp3s0f1

gnome-terminal -- bash -c "rosbag record -a --split --duration=1800 -O /home/adachi/`date +%Y%m%d%H%M`.bag; exec bash"
sleep 4





# gnome-terminal -- bash -c "roscore; exec bash"
# gnome-terminal -- bash -c " \
# rosbag record \
# /camera/accel/imu_info /camera/accel/metadata /camera/accel/sample \
# /camera/gyro/imu_info /camera/gyro/metadata /camera/gyro/sample \
# /camera/odom/metadata /camera/odom/sample /camera/realsense2_camera_manager/bond \
# /camera/tracking_module/parameter_descriptions /camera/tracking_module/parameter_updates \
# /cmd_vel /diagnostics /gnss_pose /gps_corr /imu /imu/data /joint_states \
# /joy /joy/set_feedback /left_rps /mag /motors_status /moving_ang \
# /nav/filtered_imu/data /nav/heading /nav/heading_state /nav/odom \
# /nav/status /odom /points2 /right_rps /rosout /rosout_agg /rtcm /scan \
# /tf /tf_static /ublox/fix /ublox/fix_velocity /ublox/hp_fix \
# /ublox/navclock /ublox/navheading /ublox/navhpposecef /ublox/navhpposllh \
# /ublox/navposecef /ublox/navpvt /ublox/navrelposned /ublox/navsat \
# /ublox/navstatus /velodyne_nodelet_manager/bond \
# /velodyne_nodelet_manager_driver/parameter_descriptions /velodyne_nodelet_manager_driver/parameter_updates \
# /velodyne_nodelet_manager_laserscan/parameter_descriptions /velodyne_nodelet_manager_laserscan/parameter_updates \
# /velodyne_nodelet_manager_transform/parameter_descriptions /velodyne_nodelet_manager_transform/parameter_updates \
# /velodyne_packets /velodyne_points /rgb/image_raw \
# -O /media/adachi/logSSD/debug.bag \
# ; exec bash"



# ROSパッケージを起動
echo 'Launching ROS package...'
# source ~/.bashrc
roslaunch crawler_controller crawler_controller.launch


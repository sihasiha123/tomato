#!/bin/bash

gnome-terminal -- bash -lc "sudo ip link set can0 up type can bitrate 1000000; exec bash"
sleep 3s

#gnome-terminal --tab -- bash -lc "roslaunch offline_map_processor offline_map_process.launch; exec bash"
#sleep 3s

gnome-terminal --tab -- bash -ic "conda deactivate >/dev/null 2>&1 || true; python3 ros_can_bridge.py; exec bash"
echo "ros_can_bridge.py successfully started"
sleep 2s

gnome-terminal --tab -- bash -lc "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
echo "MID360 LiDAR driver started"
sleep 3s

gnome-terminal --tab -- bash -lc "roslaunch fast_livo mapping_mid360.launch; exec bash"
echo "fast_livo successfully started"
sleep 3s

gnome-terminal --tab -- bash -lc "USE_RVIZ=\${PUTN_USE_RVIZ:-true}; roslaunch putn_launch bringup.launch use_rviz:=\$USE_RVIZ; exec bash"
echo "putn bringup successfully started"
sleep 3s

gnome-terminal --tab -- bash -ic "conda deactivate >/dev/null 2>&1 || true; python3 /home/ubuntu/Desktop/de5/robot/putn_ws/src/putn-main/src/putn/putn_mpc/scripts/local_planner.py; exec bash"
echo "local_planner.py successfully started"
sleep 2s

gnome-terminal --tab -- bash -ic "conda deactivate >/dev/null 2>&1 || true; PUTN_AUTO_START=1 python3 /home/ubuntu/Desktop/de5/robot/putn_ws/src/putn-main/src/putn/putn_mpc/scripts/controller.py; exec bash"
echo "controller.py successfully started"

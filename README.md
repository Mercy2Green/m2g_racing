# m2g_racing

## Simulator

### run_simulator

Location: home/uav/irms

URL:https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator

terminal: ./run_simulator.sh

Help in simulator: F1

### topic view

tool: rqt

if rqt can not get message:

    cd /home/m2g/uav/m2g_racing/m2g_racing
    source devel/setup.bash

URL:https://github.com/RoboMaster/IntelligentUAVChampionshipBase

uav_real_position : /airsim_node/drone_1/debug/pose_gt

circle_real_position : /airsim_node/drone_1/debug/circle_poses_gt

output_topic: type(geometry_msgs::PoseStamped)


## Control

### keyboard control

name: kb_ctrl

location: /home/m2g/uav/m2g_racing/irmb/kb_ctrl

source : source devel/setup.bash

run: roslaunch kb_ctrl kb_ctrl.launch 

URL:https://github.com/RoboMaster/IntelligentUAVChampionshipBase



## calibration

### stereo

terminal : rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 11x8 --square 0.04 right:=/airsim_node/drone_1/front_right/Scene left:=/airsim_node/drone_1/front_left/Scene


## Algorithm
 

 ### Vins

 roslaunch vins m2g_change_image_brightness.launch

 rostopic echo /vins_fusion/imu.......


 ### m2g_racing

 roslaunch m2g_racing


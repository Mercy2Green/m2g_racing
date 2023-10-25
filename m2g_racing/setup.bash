#!/bin/bash

cd /m2g_racing
source devel/setup.bash

#bash shfile/m2g_racing.sh & sleep 5

roslaunch vins m2g.launch & sleep 3

roslaunch ego_planner run_in_race.launch & sleep 10

roslaunch m2g_racing circle_2_waypoint.launch



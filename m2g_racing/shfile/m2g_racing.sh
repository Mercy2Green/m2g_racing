# sudo chmod 777 /dev/ttyACM0 & sleep 2;

source /home/m2g/uav/m2g_racing/m2g_racing/devel/setup.bash & sleep 2;

roslaunch vins m2g.launch & sleep 10;

wait;

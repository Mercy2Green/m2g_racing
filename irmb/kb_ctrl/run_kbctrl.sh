echo bash /home/m2g/uav/IntelligentUAVChampionshipSimulator/run_simulator.sh

xhost +local:docker
docker run -it --net host -e DISPLAY=$DISPLAY --name kb_ctrl --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw kb_ctrl

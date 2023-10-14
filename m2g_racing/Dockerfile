FROM osrf/ros:noetic-desktop-full-focal
ADD src /m2g_racing/src/
ADD setup.bash /
ADD shfile /m2g_racing/shfile
ADD ceres /ceres
ADD glog /glog

RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list

RUN chmod +x /setup.bash

USER root



RUN apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
 ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
 net-tools libeigen3-dev 

RUN apt install -y python3-pip ros-noetic-geographic-* geographiclib-* libgeographic-*


# RUN pip install geographiclib

# CMake
RUN apt-get -y install cmake
# google-glog + gflags
RUN apt-get -y install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
RUN apt-get -y install libatlas-base-dev
# Eigen3
RUN apt-get -y install libeigen3-dev
# SuiteSparse (optional)
RUN apt-get -y install libsuitesparse-dev libtool

RUN cd glog && ./autogen.sh && ./configure && make && sudo make install
RUN apt-get install -y liblapack-dev libsuitesparse-dev libgflags-dev libgoogle-glog-dev libgtest-dev

RUN cd ../ceres && mkdir build && cd build && cmake .. && make -j4 && make install
RUN apt-get install -y ros-noetic-ddynamic-reconfigure
RUN cd ..


ENV ROS_DISTRO noetic

WORKDIR /m2g_racing/
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps m2g_racing
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps image_pipeline
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps robot_localization
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps vins

ENTRYPOINT [ "/setup.bash" ]

#!/bin/bash

WS_NAME="ros1"

sudo apt update
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy

cd "$HOME/$WS_NAME"
source devel/setup.bash

wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY="/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so"
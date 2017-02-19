#!/bin/sh
sleep 30s
cd /home/ubuntu
echo ubuntu | sudo -S ./jetson-scripts/jetson_max_l4t.sh
cd robot-code
v4l2ctrl -d /dev/video1 -l c2017/vision/coprocessor/camera_params
echo Robot ip: $1
./bazel-bin/c2017/vision/coprocessor/main --robot_ip=$1

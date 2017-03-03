#!/bin/sh
#
# This script runs vision and all the things vision needs:
# * Turn on fan
# * Set camera parameters - minimum exposure and brightness
# * Run vision
#
# Usage: startup.sh roborio-ip
#
# To run this on startup, add the following line or something similar to /etc/rc.init
#
#     /home/ubuntu/robot-code/c2017/vision/coprocessor/startup.sh 10.16.78.22 >& /home/ubuntu/log.text &
#

# This script runs before the camera is ready, so sleep until it is ready.
sleep 30s
cd /home/ubuntu
./jetson-scripts/jetson_max_l4t.sh # Script to run fan
cd robot-code
v4l2ctrl -d /dev/video1 -l c2017/vision/coprocessor/camera_params
echo Robot ip: $1
./bazel-bin/c2017/vision/coprocessor/main --robot_ip=$1

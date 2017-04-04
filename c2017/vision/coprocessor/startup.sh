#!/bin/sh
#
# This script runs vision and all the things vision needs:
# * Set camera parameters - minimum exposure and brightness
# * Run vision
#
# Usage: startup.sh roborio-ip
#
# To run this on startup, add the following line or something similar to /etc/rc.init
#
#     /home/ubuntu/robot-code/c2017/vision/coprocessor/startup.sh 10.16.78.22 >& /home/ubuntu/log.text &
#

cd /home/ubuntu/robot-code

VISION_CAM=$(./muan/vision/find_camera_by_name.py --name LifeCam)
STREAM_CAM=$(./muan/vision/find_camera_by_name.py --name USB_Camera)

echo Robot ip: $1
echo Vision camera index: $VISION_CAM
echo Stream camera index: $STREAM_CAM

v4l2ctrl -d /dev/video$VISION_CAM -l c2017/vision/coprocessor/constants/vision_camera_params
v4l2ctrl -d /dev/video$STREAM_CAM -l c2017/vision/coprocessor/constants/stream_camera_params

mjpg_streamer -i "input_uvc.so -d /dev/video$STREAM_CAM" -o "output_http.so -p 5802" &
./bazel-bin/c2017/vision/coprocessor/main --robot_ip=$1 --camera_index=$VISION_CAM

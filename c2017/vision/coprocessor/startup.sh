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

# This line is necessary because for some reason, I don't know exactly why, everything has
# a high probability of breaking horribly for strange reasons when it isn't here. You
# might think "oh, this line seems sketchy, and I deleted it and it seemed to work."
# Well yes, it is sketchy, and if you delete it things work sometimes, but it will be a lot
# of pain when we get 10 different bugs in the vision code and need to re-discover that
# this is necessary. It might make you cry, and it makes me cry too, but just trust me,
# don't delete it. Seriously, git blame exists and we will find you.
sleep 30s

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

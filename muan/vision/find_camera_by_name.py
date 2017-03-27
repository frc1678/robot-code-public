#!/usr/bin/env python

import os
import glob
import argparse

parser = argparse.ArgumentParser(description='Find a camera\' file in /dev by the name of the camera.')
parser.add_argument('--name', required=True, help='A substring of the camera name that you want.')
args = parser.parse_args()

for camera in glob.glob("/sys/class/video4linux/video*/"):
    with open(os.path.join(camera, "name")) as camera_file:
        camera_name = camera_file.read()
        if args.name in camera_name:
            print(os.path.join("/dev/", os.path.basename(os.path.normpath(camera))))

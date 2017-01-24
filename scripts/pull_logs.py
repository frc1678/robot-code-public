#/usr/bin/env python

import os
import argparse
import subprocess

parser = argparse.ArgumentParser(description='Pull log files from the roboRIO')
parser.add_argument('--team', default='1678', help='What team number to use for the roboRIO.')
parser.add_argument('--address', help='What IP address to fetch from. Not compatible with --team')
parser.add_argument('--user', default='admin', help='What user to log in as on the roboRIO')
parser.add_argument('--path', default='/home/lvuser/logs/', help='What path to fetch logs from.')


args = parser.parse_args()

if not args.address:
    rio_address = "roborio-{}-frc.local".format(args.team)
else:
    rio_address = args.address
rio_user = args.user
rio_path = args.path
local_path = os.path.abspath(os.path.dirname(__file__)) + "/../logs/" # Write to robot-code/logs/

rsync_args = ["--verbose", "--archive", "--times", "--human-readable", "--progress"]
rsync_remote_path = "{user}@{address}:{path}".format(user=rio_user, address=rio_address, path=rio_path)
rsync_command = ["rsync"] + rsync_args + [rsync_remote_path, local_path]

subprocess.call(rsync_command)

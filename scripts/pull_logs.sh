#!/bin/bash
# How to pull log number XXX from robot YYYY:
# `./pull_logs.sh XXX YYYY`
# Get value automatically using '_' or leaving place blank
# `./pull_logs.sh _ YYYY` pulls latest log
# `./pull_logs.sh XXX` _ or `./pull_logs.sh XXX` pulls log XXX from robot currently connected
# `./pull_logs.sh` pulls latest log from currently connected robot
# Logs will be pulled to /tmp/

if [[ $# -eq 0 ]]; then
  log="_"
  robot="_"
elif [[ $# -eq 1 ]]; then
  log=$1
  robot="_"
else
  log=$1
  robot=$2
fi

if [[ $robot == "_" ]]; then
  ip4_to_robot="s/^.*10\\.([0-9]{1,3})\\.([0-9]{1,3})\\.[0-9]{1,3}.*$/\\1\\2/"
  robot=$(ifconfig | grep "inet addr:10\\." | sed -r $ip4_to_robot)
  if [[ $robot == "" ]]; then
    echo "No robot found"
    exit
  fi
  echo "Default robot $robot found"
fi

robot_user="admin@roborio-$robot-frc.local"
robot_log_dir="/media/sda1/logs"

if [[ $log == "_" ]]; then
  log=$(ssh $robot_user "ls $robot_log_dir | sort -n | tail -n 1")
  if [[ $log == "" ]]; then
    echo "No logs found"
    exit
  fi
  echo "Latest log $log found"
fi

scp -r "$robot_user:$robot_log_dir/$log" /tmp/
${BASH_SOURCE%/*}/logs/log_viewer.py /tmp/$log > /dev/null 2>&1 &

#!/bin/bash
# How to pull log number XXX from robot YYYY:
# ./pull_logs.sh XXX YYYY
# Logs will be pulled to /tmp/
scp -r "admin@roborio-$2-frc.local:/media/sda1/logs/$1/" /tmp/

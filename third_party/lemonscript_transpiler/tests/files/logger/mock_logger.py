#!/usr/bin/python
import os
import sys

parent_dir = os.path.dirname(os.path.realpath(__file__)) + "/../../../"
sys.path.append(parent_dir)
from objects.logger import Logger

if len(sys.argv) > 3:
    Logger.show_log_levels = int(sys.argv[3])

if sys.argv[1] == "error":
    Logger.error(sys.argv[2])
    sys.exit(0)
if sys.argv[1] == "warn":
    Logger.warn(sys.argv[2])
    sys.exit(0)
if sys.argv[1] == "info":
    Logger.info(sys.argv[2])
    sys.exit(0)
if sys.argv[1] == "debug":
    Logger.debug(sys.argv[2])
    sys.exit(0)

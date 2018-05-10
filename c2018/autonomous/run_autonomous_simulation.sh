#!/bin/bash
if [[ $# -ne 2 ]]; then
  echo "Usage example: run_autonomous_simulation.sh SCALE_PLUS_SWITCH RRR"
  exit
fi
# Can't just call 'system("gnuplot ...")' because of sandboxing, so output the command instead
bazel run //c2018/autonomous:autonomous_simulation -- $1 $2 | tee /dev/stderr | tail -n 1 | sh

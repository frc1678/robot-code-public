#!/bin/bash
set -e

if [[ $* == *--no-vision* ]]; then
  bazel test --test_output=errors //muan/... -- -//muan/wpilib/... -//muan/teleop/... -//muan/vision/...
else
  bazel test --test_output=errors //muan/... -- -//muan/wpilib/... -//muan/teleop/...
fi

bazel build //muan/... --cpu=roborio -- -//muan/vision/...

bazel test --test_output=errors //testbench/... -- -//testbench:frc1678 -//testbench/wpilib/... -//testbench/subsystems:subsystem_runner -//testbench/teleop/...
bazel build //testbench/... --cpu=roborio

bazel test --test_output=errors //o2016/... -- -//o2016:frc1678 -//o2016/wpilib/... -//o2016/subsystem_runner/... -//o2016/teleop/...
bazel build //o2016/... --cpu=roborio

bazel test --test_output=errors //generic_robot/... -- -//generic_robot:frc1678 -//generic_robot/wpilib/... -//generic_robot/subsystem_runner/... -//generic_robot/wpilib_update/...
bazel build //generic_robot/... --cpu=roborio

bazel test --test_output=errors //c2017/... -- -//c2017:frc1678 -//c2017/wpilib/... -//c2017/subsystems:subsystem_runner -//c2017/wpilib_update/...
bazel build //c2017/... --cpu=roborio

bazel test --test_output=errors //third_party/frc971/control_loops/drivetrain:drivetrain_lib_test

./check-format.py --no-fail

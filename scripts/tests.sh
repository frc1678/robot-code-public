#!/bin/bash
set -e
cd ${BASH_SOURCE%/*}

# We edited frc971 code enough we should test it
no_third_party="-//third_party/... //third_party/frc971/..."
no_wpilib=$(bazel query "rdeps(//..., //third_party/wpilibsuite/...)" | sed "s/^/-/")
no_vision=$(bazel query "rdeps(//..., //muan/vision/...)" | sed "s/^/-/")
no_tests=$(bazel query "kind(test, //...)" | sed "s/^/-/")
no_tools="-//tools/..."

# If "--no-vision" is present in any arguments. Use when OpenCV is not installed on host.
if (echo "$@" | grep -e --no-vision > /dev/null)
then
  host_extra_excludes="$host_extra_excludes $no_vision"
fi

bazel test //... --test_output=errors -- $no_third_party $no_wpilib $no_tools $host_extra_excludes
bazel build //... -- $no_third_party $no_vision $no_tests $no_tools
bazel build //... --cpu=roborio -- $no_third_party $no_vision $no_tests $no_tools
./cpplint/run-cpplint.sh

# If "--show-formatting" is present in any arguments
if (echo "$@" | grep -e --show-formatting > /dev/null)
then
  ./check-format.py --no-fail
fi

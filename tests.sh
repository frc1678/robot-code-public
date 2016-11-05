set -e
bazel test --test_output=errors //muan/... -- -//muan/wpilib/...
bazel build //muan/... --cpu=roborio
bazel test --test_output=errors //o2016/... -- -//o2016:frc1678
bazel build //o2016/... --cpu=roborio
#./check-format.py

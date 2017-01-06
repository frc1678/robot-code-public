set -e
# Our code is matched my the regular expression (muan|generic_robot|o\d{4}|c\d{4})
OUR_CODE='^//(muan|generic_robot|o\\d{4}|c\\d{4})'
bazel test --test_output=errors //muan/... --output_filter=$OUR_CODE -- -//muan/wpilib/... -//muan/teleop/...
bazel build //o2016/... --cpu=roborio --output_filter=$OUR_CODE
bazel test --test_output=errors //o2016/... --output_filter=$OUR_CODE -- -//o2016:frc1678 -//o2016/wpilib/... -//o2016/subsystem_runner/... -//o2016/teleop/...
bazel build //muan/... --cpu=roborio --output_filter=$OUR_CODE
bazel test --test_output=errors //generic_robot/... --output_filter=$OUR_CODE -- -//generic_robot:frc1678 -//generic_robot/wpilib/... -//generic_robot/subsystem_runner/... -//generic_robot/teleop/...
bazel build //generic_robot/... --output_filter=$OUR_CODE --cpu=roborio
./check-format.py --no-fail

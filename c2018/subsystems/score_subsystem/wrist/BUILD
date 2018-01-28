load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "wrist_python",
    srcs = ["wrist_controller.py"],
    main = "wrist_controller.py",
    deps = ["//muan/control:python_controls"],
)
genrule(
  name = "wrist_genrule",
  srcs = [],
  outs = [
      "wrist_constants.h",
      "wrist_constants.cpp",
  ],
  cmd = "$(location :wrist_python) $(location wrist_constants.h) $(location wrist_constants.cpp)",
  tools = [":wrist_python"],
)
cc_library(
  name = "wrist_controller",
  srcs = ["wrist.cpp"],
  hdrs = ["wrist.h"],
  deps = [
    "//c2018/subsystems/score_subsystem:score_subsystem_queues",
    "//c2018/subsystems/score_subsystem:score_subsystem_proto",
    ":wrist_constants",
    "//muan/control/calibration:hall_calibration",
    "//muan/control:state_space",
    "//muan/units:units",
    "//muan/utils:math_utils",
    "//muan/wpilib:wpilib_queues",
    "//third_party/aos/common/util:trapezoid_profile",
    "//third_party/google/googletest:googletest",
  ],
)

cc_library(
    name = "wrist_constants",
    srcs = ["wrist_constants.cpp"],
    hdrs = ["wrist_constants.h"],
    deps = ["//third_party/eigen"],
)

cc_test(
    name = "wrist_test",
    srcs = ["wrist_test.cpp"],
    deps = [
      ":wrist_controller",
      "//third_party/aos/common/util:trapezoid_profile",
      "//third_party/google/googletest:googletest",
    ],
)

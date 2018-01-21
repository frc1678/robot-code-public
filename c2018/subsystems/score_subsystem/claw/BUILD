load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "claw_python",
    srcs = ["claw_controller.py"],
    main = "claw_controller.py",
    deps = ["//muan/control:python_controls"],
)
genrule(
  name = "claw_genrule",
  srcs = [],
  outs = [
      "claw_constants.h",
      "claw_constants.cpp",
  ],
  cmd = "$(location :claw_python) $(location claw_constants.h) $(location claw_constants.cpp)",
  tools = [":claw_python"],
)
cc_library(
  name = "claw_controller",
  srcs = ["claw.cpp"],
  hdrs = ["claw.h"],
  deps = [
    "//c2018/subsystems/score_subsystem:score_subsystem_queues",
    "//c2018/subsystems/score_subsystem:score_subsystem_proto",
    ":claw_constants",
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
    name = "claw_constants",
    srcs = ["claw_constants.cpp"],
    hdrs = ["claw_constants.h"],
    deps = ["//third_party/eigen"],
)

cc_test(
    name = "claw_test",
    srcs = ["claw_test.cpp"],
    deps = [
      "//c2018/subsystems/score_subsystem:score_subsystem_queues",
      ":claw_constants",
      ":claw_controller",
      "//muan/control/calibration:hall_calibration",
      "//muan/control:state_space",
      "//muan/utils:math_utils",
      "//muan/units:units",
      "//muan/wpilib:wpilib_queues",
      "//third_party/aos/common/util:trapezoid_profile",
      "//third_party/google/googletest:googletest",
    ],
)

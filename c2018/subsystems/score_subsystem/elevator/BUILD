package(default_visibility = ["//visibility:public"])

cc_library(
  name = "elevator_controller",
  srcs = ["elevator_controller.cpp"],
  hdrs = ["elevator_controller.h"],
  deps = [
      ":elevator_constants",
      "//c2018/subsystems/score_subsystem:score_subsystem_queues",
      "//c2018/subsystems/score_subsystem:score_subsystem_proto",
      "//muan/queues:queue_manager",
      "//muan/control:state_space",
      "//muan/wpilib:wpilib_queues",
      "//third_party/aos/common/util:trapezoid_profile",
      "//muan/control/calibration:hall_calibration",
      "//muan/utils:math_utils"
  ]
)

cc_test(
  name = "elevator_controller_test",
  srcs = ["elevator_controller_test.cpp"],
  deps = [
      ":elevator_controller",
      "//third_party/google/googletest",
  ]
)

cc_library(
    name = "elevator_constants",
    srcs = [":elevator_constants.cpp"],
    hdrs = [":elevator_constants.h"],
    deps = ["//third_party/eigen"],
)

genrule(
    name = "elevator_genrule",
    srcs = [],
    outs = [
        "elevator_constants.h",
        "elevator_constants.cpp",
    ],
    cmd = "$(location :elevator_python) $(location elevator_constants.h) $(location elevator_constants.cpp)",
    tools = ["elevator_python"],
)

py_binary(
    name = "elevator_python",
    srcs = ["elevator_controller.py"],
    main = "elevator_controller.py",
    deps = ["//muan/control:python_controls"],
)

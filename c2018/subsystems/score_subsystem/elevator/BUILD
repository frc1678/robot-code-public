package(default_visibility = ["//visibility:public"])

cc_library(
    name = "elevator",
    srcs = ["elevator.cpp"],
    hdrs = ["elevator.h"],
    deps = [
        ":elevator_constants",
        "//c2018/subsystems/score_subsystem:score_subsystem_proto",
        "//c2018/subsystems/score_subsystem:score_subsystem_queues",
        "//muan/control:state_space",
        "//muan/control/calibration:hall_calibration",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/utils:math_utils",
        "//muan/wpilib:wpilib_queues",
        "//third_party/aos/common/util:trapezoid_profile",
    ],
)

cc_test(
    name = "elevator_test",
    srcs = ["elevator_test.cpp"],
    deps = [
        ":elevator",
        "//third_party/google/googletest",
    ],
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
    srcs = ["elevator.py"],
    main = "elevator.py",
    deps = ["//muan/control:python_controls"],
)

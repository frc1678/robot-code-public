package(default_visibility = ["//visibility:public"])

cc_library(
    name = "arm_interface",
    srcs = ["arm_interface.cpp"],
    hdrs = ["arm_interface.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/queues:queue_manager",
        "//o2018/subsystems/arm:arm_queues",
        "//muan/wpilib:can_wrapper",
        "//muan/phoenix:talon_wrapper",
        "//muan/phoenix:victor_wrapper",
    ],
)

cc_library(
    name = "drive_interface",
    srcs = ["drive_interface.cpp"],
    hdrs = ["drive_interface.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:can_wrapper",
        "//muan/phoenix:talon_wrapper",
        "//muan/phoenix:victor_wrapper",
    ],
)

cc_library(
    name = "interface_runner",
    srcs = ["interface_runner.cpp"],
    hdrs = ["interface_runner.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":arm_interface",
        ":drive_interface",
    ],
)

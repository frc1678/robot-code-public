cc_library(
    name = "teleop",
    srcs = ["teleop.cpp"],
    hdrs = ["teleop.h"],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2019/autonomous:autonomous_queues",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/teleop:joystick",
        "//muan/utils:threading_utils",
        "//muan/wpilib:ds_sender",
    ],
)

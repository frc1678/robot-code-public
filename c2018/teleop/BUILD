cc_library(
    name = "main",
    srcs = ["main.cpp"],
    hdrs = ["main.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/logging:logger",
        "//muan/teleop:joystick",
        "//muan/wpilib:ds_sender",
        "//muan/queues:queue_manager",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_queues",
    ],
    restricted_to = ["//tools:roborio"],
)

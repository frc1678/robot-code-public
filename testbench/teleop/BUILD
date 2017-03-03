cc_library(
    name = "teleop",
    srcs = ["teleop.cpp"],
    hdrs = ["teleop.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/actions:drivetrain_action",
        "//muan/teleop:joystick",
        "//muan/wpilib:ds_sender",
        "//testbench/queue_manager",
        "//testbench/subsystems/drivetrain:drivetrain_base",
    ],
)

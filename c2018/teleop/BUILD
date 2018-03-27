cc_library(
    name = "main",
    srcs = ["main.cpp"],
    hdrs = ["main.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2018/subsystems/climber:climber_queues",
        "//c2018/subsystems/score_subsystem:score_subsystem_queues",
        "//c2018/subsystems/lights:lights_queues",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
        "//muan/teleop:joystick",
        "//muan/wpilib:ds_sender",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_queues",
    ],
    restricted_to = ["//tools:roborio"],
)

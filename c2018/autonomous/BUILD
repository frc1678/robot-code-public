cc_library(
    name = "autonomous",
    srcs = ["autonomous.cpp"],
    hdrs = ["autonomous.h"],
    deps = [
        "//c2018/subsystems/drivetrain:drivetrain_base",
        "//c2018/subsystems/score_subsystem:score_subsystem_queues",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
        "//muan/wpilib:wpilib_queues",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
    ],
    visibility = ["//visibility:public"],
)

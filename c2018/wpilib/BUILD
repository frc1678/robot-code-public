cc_library(
    name = "wpilib_interface",
    srcs = [
        "climber_interface.cpp",
        "drivetrain_interface.cpp",
        "score_interface.cpp",
        "wpilib_interface.cpp",
    ],
    hdrs = [
        "climber_interface.h",
        "drivetrain_interface.h",
        "score_interface.h",
        "wpilib_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//c2018/subsystems/climber:climber_queues",
        "//c2018/subsystems/score_subsystem:score_subsystem_queues",
        "//muan/queues:queue_manager",
        "//muan/wpilib:can_wrapper",
        "//muan/wpilib/gyro:gyro",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_queues",
        "//third_party/wpilibsuite:wpilib",
    ],
)

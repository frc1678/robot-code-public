cc_library(
    name = "queue_manager",
    srcs = ["queue_manager.cpp"],
    hdrs = ["queue_manager.h"],
    deps = [
        "//muan/queues:queues",
        "//muan/proto:stack_proto",
        "//muan/wpilib:wpilib_queues",
        "//muan/wpilib/gyro:gyro_queues",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
        "//muan/logging:logger",
        "//c2017/subsystems/superstructure/shooter:shooter_queues",
        "//c2017/subsystems/superstructure/magazine:magazine_queues",
        "//c2017/subsystems/superstructure/trigger:trigger_queues",
        "//c2017/subsystems/superstructure/ground_gear_intake:ground_gear_intake_queues",
        "//c2017/subsystems/superstructure/ground_ball_intake:ground_ball_intake_queues",
        "//c2017/subsystems/superstructure/climber:climber_queues",
        "//c2017/subsystems/superstructure:intake_group_queues",
        "//c2017/subsystems/superstructure:shooter_group_queues",
        "//c2017/vision:vision_queues",
        "//c2017/wpilib:wpilib_queues",
        "//c2017/webdash:webdash_queues",
        "//c2017/subsystems/lights:lights_queues",
        "//c2017/vision:vision_queues"
    ],
    visibility = ["//visibility:public"]
)

cc_test(
    name = "queue_manager_test",
    srcs = ["test_queue_manager.cpp"],
    deps = [
        ":queue_manager",
        "//third_party/google/googletest",
    ],
)

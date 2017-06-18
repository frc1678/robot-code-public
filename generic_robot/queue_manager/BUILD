cc_library(
    name = "queue_manager",
    srcs = ["queue_manager.cpp"],
    hdrs = ["queue_manager.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/logging:logger",
        "//muan/proto:stack_proto",
        "//muan/teleop:joystick_queues",
        "//muan/queues",
        "//muan/webdash:server",
        "//muan/wpilib:wpilib_queues",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
    ],
)

cc_test(
    name = "queue_manager_test",
    srcs = ["test_queue_manager.cpp"],
    deps = [
        ":queue_manager",
        "//third_party/google/googletest",
    ],
)

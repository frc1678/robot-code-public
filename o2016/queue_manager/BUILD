cc_library(
    name = "queue_manager",
    srcs = ["queue_manager.cpp"],
    hdrs = ["queue_manager.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/proto:stack_proto",
        "//muan/queues",
        "//muan/wpilib:wpilib_queues",
        "//o2016/subsystems/drivetrain:queue_types",
        "//o2016/subsystems/superstructure/catapult:catapult_queues",
        "//o2016/subsystems/superstructure/intake:intake_queues",
        "//o2016/subsystems/superstructure/secondaries:secondaries_queues",
        "//o2016/subsystems/superstructure/turret:turret_queues",
    ],
)

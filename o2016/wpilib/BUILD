cc_library(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cpp",
    ],
    hdrs = [
        "wpilib_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/wpilib:can_wrapper",
        "//o2016/queue_manager",
        "//o2016/subsystems/drivetrain:queue_types",
        "//o2016/subsystems/superstructure/catapult:catapult_queues",
        "//o2016/subsystems/superstructure/intake:intake_queues",
        "//o2016/subsystems/superstructure/secondaries:secondaries_queues",
        "//o2016/subsystems/superstructure/turret:turret_queues",
        "//third_party/wpilibsuite:wpilib",
    ],
)

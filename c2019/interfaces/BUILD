package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drive_interface",
    srcs = ["drive_interface.cpp"],
    hdrs = ["drive_interface.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:can_wrapper",
        "//third_party/phoenixsuite:phoenix",
    ],
)

cc_library(
    name = "superstructure_interface",
    srcs = ["superstructure_interface.cpp"],
    hdrs = ["superstructure_interface.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/queues:queue_manager",
        "//c2019/subsystems/superstructure:superstructure_queues",
        "//muan/wpilib:can_wrapper",
        "//third_party/phoenixsuite:phoenix",
    ],
)

cc_library(
    name = "interface_runner",
    srcs = ["interface_runner.cpp"],
    hdrs = ["interface_runner.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":drive_interface",
        ":superstructure_interface",
    ],
)

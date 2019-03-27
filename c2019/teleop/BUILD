load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teleop",
    srcs = ["teleop.cpp"],
    hdrs = ["teleop.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2019/commands:commands",
        "//c2019/subsystems/superstructure:superstructure",
        "//c2019/subsystems/superstructure:superstructure_queues",
 	    "//c2019/subsystems/limelight:limelight_queues",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/teleop:joystick",
        "//muan/utils:threading_utils",
        "//muan/wpilib:ds_sender",
    ],
    restricted_to = ["//tools:roborio"],
)

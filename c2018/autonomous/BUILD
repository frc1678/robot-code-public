cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    deps = [
        ":switch_and_scale",
        ":switch_only",
        ":scale_only",
        ":backside_switch",
        ":drive",
        ":none",
        ":sneak",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "autonomous_base",
    srcs = ["autonomous_base.cpp"],
    hdrs = ["autonomous_base.h"],
    deps = [
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
        "//muan/wpilib:wpilib_queues",
        "//c2018/subsystems/drivetrain:drivetrain_base",
        "//c2018/subsystems/score_subsystem:score_subsystem_queues",
        "//third_party/google/googletest:googletest",
    ],
    visibility = ["//visibility:public"],
)

# cc_binary(
#     name = "autonomous_simulation",
#     srcs = ["autonomous_simulation.cpp"],
#     deps = [
#         ":autonomous_runner",
#         "//third_party/frc971/control_loops/drivetrain:drivetrain_simulation",
#         "//muan/utils:string_utils",
#         "//c2018/subsystems/drivetrain:drivetrain_base",
#         "//c2018/subsystems/drivetrain:polydrivetrain_plants",
#     ],
# )

cc_test(
    name = "autonomous_base_test",
    srcs = ["autonomous_base_test.cpp"],
    deps = [
        ":autonomous_base",
        "//third_party/google/googletest:googletest",
    ],
)

cc_library(
    name = "switch_and_scale",
    srcs = ["switch_and_scale.cpp"],
    hdrs = ["switch_and_scale.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "switch_only",
    srcs = ["switch_only.cpp"],
    hdrs = ["switch_only.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "scale_only",
    srcs = ["scale_only.cpp"],
    hdrs = ["scale_only.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "backside_switch",
    srcs = ["backside_switch.cpp"],
    hdrs = ["backside_switch.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "drive",
    srcs = ["drive.cpp"],
    hdrs = ["drive.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "none",
    srcs = ["none.cpp"],
    hdrs = ["none.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "sneak",
    srcs = ["sneak.cpp"],
    hdrs = ["sneak.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

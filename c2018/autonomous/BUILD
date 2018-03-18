cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    deps = [
        ":switch_and_scale",
        ":switch_only",
        ":scale_only",
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

# TODO(Lyra): Make this work with updated framework
cc_test(
    name = "autonomous_simulation",
    srcs = ["autonomous_simulation.cpp"],
    deps = [
        ":autonomous_runner",
        "//third_party/google/googletest:googletest",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_simulation",
        "//c2018/subsystems/drivetrain:drivetrain_base",
        "//c2018/subsystems/drivetrain:polydrivetrain_plants",
    ],
)

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
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "switch_only",
    srcs = ["switch_only.cpp"],
    hdrs = ["switch_only.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "scale_only",
    srcs = ["scale_only.cpp"],
    hdrs = ["scale_only.h"],
    deps = [
        ":autonomous_base",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

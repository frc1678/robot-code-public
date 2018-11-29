cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
        ":test_auto",
        ":none",
        ":drive_straight",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
    ],
)

cc_library(
    name = "autonomous_base",
    srcs = ["autonomous_base.cpp"],
    hdrs = ["autonomous_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:wpilib_queues",
        "//o2018/subsystems/arm:arm_queues",
    ],
)

cc_test(
    name = "autonomous_base_test",
    srcs = ["autonomous_base_test.cpp"],
    deps = [
        ":autonomous_base",
        "//third_party/google/googletest",
    ],
)

cc_library(
    name = "test_auto",
    srcs = ["test_auto.cpp"],
    hdrs = ["test_auto.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)

cc_library(
    name = "none",
    srcs = ["none.cpp"],
    hdrs = ["none.h"],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)

cc_library(
    name = "drive_straight",
    srcs = ["drive_straight.cpp"],
    hdrs = ["drive_straight.h"],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)

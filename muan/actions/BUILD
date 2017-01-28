cc_library(
    name = "drivetrain_action",
    srcs = ["drivetrain_action.cpp"],
    hdrs = ["drivetrain_action.h"],
    visibility = ["//visibility:public"],
    deps = ["//third_party/frc971/control_loops/drivetrain:drivetrain_queues"],
)

cc_test(
    name = "drivetrain_action_test",
    srcs = ["drivetrain_action_test.cpp"],
    deps = [
        ":drivetrain_action",
        "//third_party/google/googletest",
    ],
)

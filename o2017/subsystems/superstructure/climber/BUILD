load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "climber",
    srcs = ["climber.cpp"],
    hdrs = ["climber.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//o2017/subsystems/superstructure:superstructure_queues",
         "//o2017/queue_manager",
         "//muan/units",
         "//muan/utils:monitor",
         "//muan/utils:history",
         "//muan/wpilib:wpilib_queues",
    ],
)

cc_test(
    name = "climber_test",
    srcs = ["climber_test.cpp"],
    deps = [
        ":climber",
        "//third_party/google/googletest",
    ],
)

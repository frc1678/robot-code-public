cc_library(
    name = "teleop",
    srcs = ["teleop.cpp"],
    hdrs = ["teleop.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/wpilib:ds_sender",
        "//muan/utils:threading_utils",
    ],
    restricted_to = ["//tools:roborio"],
)

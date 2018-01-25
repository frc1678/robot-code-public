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
        "//muan/wpilib/gyro",
        "//testbench/queue_manager",
        "//third_party/wpilibsuite:wpilib",
    ],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "subsystem_runner",
    srcs = ["subsystem_runner.cpp"],
    hdrs = ["subsystem_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//o2016/subsystems/superstructure/secondaries",
        "//o2016/wpilib:wpilib_interface",
        "//muan/utils:threading_utils",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
    ],
    restricted_to = ["//tools:roborio"],
)

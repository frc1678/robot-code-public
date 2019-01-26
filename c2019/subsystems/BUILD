cc_library(
    name = "subsystem_runner",
    srcs = ["subsystem_runner.cpp"],
    hdrs = ["subsystem_runner.h"],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2019/interfaces:interface_runner",
        "//c2019/subsystems/drivetrain:drivetrain_base",
        "//c2019/subsystems/superstructure",
        "//muan/subsystems/drivetrain",
        "//muan/utils:threading_utils",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
    ],
)

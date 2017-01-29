cc_library(
    name = "subsystem_runner",
    srcs = ["subsystem_runner.cpp"],
    hdrs = ["subsystem_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//testbench/subsystems/drivetrain:drivetrain_base",
        "//testbench/wpilib:wpilib_interface",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
    ],
)

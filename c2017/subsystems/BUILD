cc_library(
    name = "subsystem_runner",
    srcs = ["subsystem_runner.cpp"],
    hdrs = ["subsystem_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2017/subsystems/drivetrain:drivetrain_base",
        "//c2017/subsystems/superstructure:superstructure_subsystem",
        "//c2017/wpilib:wpilib_interface",
        "//c2017/subsystems/lights:lights",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
    ],
)

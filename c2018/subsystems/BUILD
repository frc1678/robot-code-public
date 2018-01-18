cc_library(
    name = "subsystem_runner",
    srcs = ["subsystem_runner.cpp"],
    hdrs = ["subsystem_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2018/subsystems/drivetrain:drivetrain_base",
        "//c2018/subsystems/climber:climber_subsystem",
        "//c2018/wpilib:wpilib_interface",
        "//muan/utils:threading_utils",
        "//muan/queues:queue_manager",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
	"//c2018/subsystems/climber:climber_subsystem",
    ],
)

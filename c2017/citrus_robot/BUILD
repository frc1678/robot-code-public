cc_library(
    name = "main",
    srcs = ["main.cpp"],
    hdrs = ["main.h"],
    visibility = ["//visibility:public"],
    deps = [
        # Joystick needs to be first because of janky WPILib build. I don't know
        # why it does this. Possibly a bazel bug.
        # TODO(Kyle) Figure this out and fix it.
        "//muan/teleop:joystick",
        "//c2017/lemonscript",
        "//c2017/queue_manager",
        "//c2017/subsystems/drivetrain:drivetrain_base",
        "//third_party/lemonscript",
    ],
)

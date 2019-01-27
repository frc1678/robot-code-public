load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = [
        "main.cpp",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//c2019/subsystems:subsystem_runner",
        "//c2019/teleop:teleop",
        "//c2019/webdash",
        "//third_party/gflags",
        "//third_party/wpilibsuite:wpilib",
    ],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)

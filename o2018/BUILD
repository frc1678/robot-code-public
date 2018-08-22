load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = [
        "main.cpp",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//o2018/subsystems:subsystem_runner",
        "//o2018/autonomous:autonomous_runner",
        "//o2018/teleop:teleop",
        "//third_party/gflags",
        "//third_party/wpilibsuite:wpilib",
    ],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)

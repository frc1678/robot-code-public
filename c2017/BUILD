load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = ["main.cpp"],
    deps = [
        "//c2017/subsystems:subsystem_runner",
        "//c2017/citrus_robot:main",
        "//third_party/wpilibsuite:wpilib",
    ],
    data = [
        "//c2017/lemonscript/auto:auto_modes",
    ]
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)

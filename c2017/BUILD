load('//muan/deploy:deploy.bzl', 'muan_deploy')

cc_binary(
  name = "frc1678",
  deps = [
    "//third_party/wpilibsuite:wpilib",
    "//c2017/subsystems:subsystem_runner",
    "//c2017/wpilib_update:main"
  ],
  srcs = ["main.cpp"],
)

muan_deploy(
  main = ':frc1678',
  name = 'deploy',
)

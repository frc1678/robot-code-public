cc_library(
  name = "drivetrain",
  srcs = ["drivetrain.cpp"],
  hdrs = ["drivetrain.h"],
  visibility = ["//visibility:public"]
)

cc_binary(
  name = "main",
  srcs = ["main.cpp", "main.h"],
  deps = [":drivetrain",
          "//third_party/wpilibsuite:wpilib"],
  visibility = ["//visibility:public"]
)

cc_test(
  name = "test",
  srcs = ["test.cpp"],
  deps = ["//third_party/google/googletest:googletest", "drivetrain"],
  visibility = ["//visibility:public"]
)

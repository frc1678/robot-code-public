load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
  name = "winch",
  srcs = ["//c2018/subsystems/climber/winch:winch.cpp"],
  hdrs = ["//c2018/subsystems/climber/winch:winch.h"],
  visibility = ["//visibility:public"],
  deps = [],
)

cc_test(
  name = "winch_test",
  srcs = ["//c2018/subsystems/climber/winch:winch_test.cpp"],
  deps = [ 
      ":winch",
      "//third_party/google/googletest",
  ],
)

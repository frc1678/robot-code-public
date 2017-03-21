licenses(["notice"])

cc_library(
  name = "pbjson",
  visibility = ["//visibility:public"],
  srcs = glob(["*.cpp"]),
  hdrs = glob(["*.h*"]) + glob(["rapidjson/**/*.h"]),
  deps = [
      "//third_party/google/protobuf:protobuf"
  ]
)

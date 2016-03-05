package(default_visibility = ["//visibility:public"])

cc_library(
  name = "muan",
  deps = [":utils", ":statemachine", ":logging", ":multithreading", ":control", "//muan/unitscpp"]
)

cc_library(
  name = "utils",
  srcs = glob(["utils/**/*.cpp"]),
  hdrs = glob(["utils/**/*.h"]),
  deps = ["//muan/unitscpp"]
)
cc_library(
  name = "statemachine",
  srcs = glob(["statemachine/**/*.cpp"]),
  hdrs = glob(["statemachine/**/*.h"]),
)

cc_library(
  name = "logging",
  srcs = glob(["logging/**/*.cpp"]),
  hdrs = glob(["logging/**/*.h"]),
)

cc_library(
  name = "multithreading",
  srcs = glob(["multithreading/**/*.cpp"]),
  hdrs = glob(["multithreading/**/*.h"]),
  deps = ["//muan/unitscpp", ":utils"],
)

cc_library(
  name = 'control',
  srcs = glob(["control/**/*.cpp"]),
  hdrs = glob(["control/**/*.h"]),
  deps = ["//muan/unitscpp", ":utils"]
)

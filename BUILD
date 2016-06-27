package(default_visibility = ["//visibility:public"])

cc_library(
  name = "muan",
  deps = [":utils", ":statemachine", ":logging", ":multithreading", ":control", "@third_party//unitscpp"]
)

cc_library(
  name = "utils",
  srcs = glob(["utils/**/*.cpp"]),
  hdrs = glob(["utils/**/*.h"]),
  deps = ["@third_party//unitscpp"]
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
  deps = [":utils"]
)

cc_library(
  name = "multithreading",
  srcs = glob(["multithreading/**/*.cpp"]),
  hdrs = glob(["multithreading/**/*.h"]),
  deps = ["@third_party//unitscpp", ":utils"],
)

cc_library(
  name = 'control',
  srcs = glob(["control/**/*.cpp"]),
  hdrs = glob(["control/**/*.h"]),
  deps = ["@third_party//unitscpp", ":utils"]
)

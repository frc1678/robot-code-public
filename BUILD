package(default_visibility = ["//visibility:public"])

cc_library(
	name = "muan",
	deps = [":utils", ":statemachine", ":logging", ":multithreading"]
)

cc_library(
	name = "utils",
	srcs = glob(["utils/**/*.cpp"]),
	includes = ["utils"],
	hdrs = glob(["utils/**/*.h"]),
  deps = [":unitscpp"]
)

cc_library(
	name = "statemachine",
	srcs = glob(["statemachine/**/*.cpp"]),
	includes = ["statemachine"],
	hdrs = glob(["statemachine/**/*.h"]),
)

cc_library(
  name = "logging",
  srcs = glob(["logging/**/*.cpp"]),
  hdrs = glob(["logging/**/*.h"]),
  includes = ["logging"]
)

cc_library(
  name = "multithreading",
  srcs = glob(["multithreading/**/*.cpp"]),
  hdrs = glob(["multithreading/**/*.h"]),
  includes = ["multithreading"],
  deps = [":unitscpp", ":utils"],
)

cc_library(
  name = 'unitscpp',
  srcs = [],
  hdrs = ['unitscpp/unitscpp.h'],
  visibility = ["//visibility:public"],
)

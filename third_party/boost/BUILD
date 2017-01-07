licenses(["notice"])

boost_path = "upstream"

load("//third_party/boost:boost.bzl", "boost_library")

boost_library(
  name = "align",
)

boost_library(
  name = "assert",
)

boost_library(
  name = "concept_check",
)

boost_library(
  name = "config",
)

boost_library(
  name = "core",
)

boost_library(
  name = "detail",
)

boost_library(
  name = "filesystem",
  srcs = glob([
    boost_path + "/filesystem/src/*.cpp",
  ]),
  deps = [
    ":config",
    ":functional",
    ":io",
    ":iterator",
    ":range",
    ":smart_ptr",
    ":system",
    ":type_traits",
  ],
)

boost_library(
  name = "functional",
  deps = [
    ":detail",
  ],
)

boost_library(
  name = "iterator",
  deps = [
    ":detail",
    ":static_assert",
  ],
)

boost_library(
  name = "io",
)

boost_library(
  name = "move",
)

boost_library(
  name = "mpl",
  deps = [
    ":preprocessor",
    ":move",
  ]
)

boost_library(
  name = "optional",
)

boost_library(
  name = "predef",
)

boost_library(
  name = "preprocessor",
)

boost_library(
  name = "range",
  deps = [
    ":concept_check",
    ":optional",
  ]
)

boost_library(
  name = "smart_ptr",
  deps = [
    ":align",
    ":core",
    ":predef",
    ":throw_exception",
    ":utility",
  ],
)

boost_library(
  name = "static_assert",
)

boost_library(
  name = "system",
  srcs = glob([
    boost_path + "/system/src/*.cpp",
  ]),
  deps = [
    ":assert",
    ":config",
    ":core",
    ":predef",
  ],
)

boost_library(
  name = "throw_exception",
)

boost_library(
  name = "type_traits",
  deps = [
    ":core",
    ":mpl",
    ":static_assert",
  ]
)

boost_library(
  name = "utility",
)

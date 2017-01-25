licenses(["notice"])

# DON'T USE THIS!
# This is the C version of mongoose. You probably want mongoose_cpp, the C++
# wrapper for this.
cc_library(
  name = "mongoose",
  srcs = ["upstream/mongoose.c"],
  hdrs = ["upstream/mongoose.h"]
)

cc_library(
  name = "mongoose_cpp",
  srcs = glob(["upstream/*.cpp"]),
  hdrs = glob(["upstream/*.h*", "upstream/json/*.h"]),
  deps = ["mongoose"],
  copts = ["-DHAS_JSONCPP=OFF"]
)

cc_binary(
  name = "example",
  srcs = ["examples/example.cpp"],
  deps = [":mongoose_cpp"],
  linkopts = ["-lpthread"]
)

# For testing only
cc_binary(
  name = 'main',
  srcs = ['main.cpp'],
  deps = [':reader'],
  linkopts = ['-lpthread']
)

cc_library(
  name = 'reader',
  srcs = ['reader.cpp'],
  hdrs = ['reader.h'],
  deps = [
    '//third_party/aos/vision/events:udp',
    '//muan/vision:vision_queues',
    '//third_party/aos/linux_code:init',
    '//third_party/aos/common/util:phased_loop',
    '//third_party/aos/common:time'
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = "vision_subsystem",
  srcs = ["vision.cpp"],
  hdrs = ["vision.h"],
  deps = [
    "//c2017/queue_manager",
    "//c2017/subsystems/drivetrain:drivetrain_base",
    "//muan/actions:drivetrain_action"
  ],
  visibility = ['//visibility:public']
)

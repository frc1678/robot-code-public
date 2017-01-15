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
    '//muan/vision:vision_queues'
  ]
)

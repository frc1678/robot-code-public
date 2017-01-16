cc_binary(
  name = 'main',
  srcs = ['main.cpp'],
  deps = [':sender', ':vision'],
  linkopts = ['-lpthread']
)

cc_library(
  name = 'sender',
  srcs = ['sender.cpp'],
  hdrs = ['sender.h'],
  deps = [
    '//third_party/aos/vision/events:udp',
    '//muan/vision:vision_queues'
  ]
)

cc_library(
  name = 'vision',
  srcs = ['vision.cpp'],
  hdrs = ['vision.h'],
  deps = [
    ':sender',
    '//muan/vision:vision'
  ],
  copts = ['-Wno-error=unused-parameter'],
  linkopts = ['-lopencv_video', '-lopencv_highgui']
)

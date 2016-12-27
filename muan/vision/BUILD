cc_binary(
  name = 'vision_example',
  srcs = ['vision_example.cpp'],
  deps = [
    '//third_party/opencv:opencv',
    ':vision'
  ],
  data = ['captured.avi']
)

cc_library(
  name = 'vision',
  srcs = ['vision.cpp'],
  hdrs = ['vision.h'],
  deps = ['//third_party/opencv:opencv']
)

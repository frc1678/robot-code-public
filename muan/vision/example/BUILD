cc_binary(
  name = 'vision_example',
  srcs = ['vision_example.cpp'],
  deps = [
    #'//third_party/opencv:opencv',
    '//muan/vision:vision'
  ],
  data = ['captured.avi'],
  linkopts = ['-lopencv_video', '-lopencv_highgui']
)

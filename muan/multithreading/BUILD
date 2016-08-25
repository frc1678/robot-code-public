package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'updateable',
  hdrs = ['updateable.h'],
  srcs = ['updateable.cpp'],
  deps = ['//muan/utils:timing'],
  linkopts = ['-lpthread']
)

cc_test(
  name = 'updateable_test',
  srcs = ['multithreading_test.cpp'],
  deps = [
    ':updateable',
    '//third_party/google/googletest:googletest' 
  ]
)

licenses(['notice'])

cc_library(
  name = 'googletest',
  visibility = ['//visibility:public'],
  srcs = [
    'gtest_main.cc',
  ],
  deps = [
    '//third_party/google/googletest',
  ],
  testonly = True,
)

cc_library(
  name = 'prevent_exit',
  visibility = ['//visibility:public'],
  srcs = [
    'prevent_exit.cc',
  ],
  hdrs = [
    'prevent_exit.h',
  ],
  deps = [
    '//third_party/aos/common:check',
  ],
  testonly = True,
)

cc_library(
  name = 'random_seed',
  visibility = ['//visibility:public'],
  srcs = [
    'random_seed.cc',
  ],
  hdrs = [
    'random_seed.h',
  ],
  testonly = True,
)

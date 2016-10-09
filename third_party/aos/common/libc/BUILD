package(default_visibility = ['//visibility:public'])
licenses(['notice'])

cc_library(
  name = 'aos_strsignal',
  srcs = [
    'aos_strsignal.cc',
  ],
  hdrs = [
    'aos_strsignal.h',
  ],
  deps = [
    '//third_party/aos/common:check'
  ],
)

cc_test(
  name = 'aos_strsignal_test',
  srcs = [
    'aos_strsignal_test.cc',
  ],
  deps = [
    ':aos_strsignal',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'dirname',
  srcs = [
    'dirname.cc',
  ],
  hdrs = [
    'dirname.h',
  ],
)

cc_test(
  name = 'dirname_test',
  srcs = [
    'dirname_test.cc',
  ],
  deps = [
    ':dirname',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'aos_strerror',
  srcs = [
    'aos_strerror.cc',
  ],
  hdrs = [
    'aos_strerror.h',
  ],
)

cc_test(
  name = 'aos_strerror_test',
  srcs = [
    'aos_strerror_test.cc',
  ],
  deps = [
    ':aos_strerror',
    '//third_party/aos/testing:googletest',
  ],
)

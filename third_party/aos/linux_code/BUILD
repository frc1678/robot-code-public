package(default_visibility = ['//visibility:public'])
licenses(['notice'])

cc_binary(
  name = 'dump_rtprio',
  srcs = [
    'dump_rtprio.cc',
  ],
  deps = [
    '//third_party/aos/common:die',
    '//third_party/aos/common:check',
    '//third_party/aos/common:time',
  ],
)

cc_library(
  name = 'queue',
  visibility = ['//third_party/aos/common:__pkg__'],
  hdrs = [
    'queue-tmpl.h',
  ],
)

cc_library(
  name = 'complex_thread_local',
  srcs = [
    'complex_thread_local.cc',
  ],
  hdrs = [
    'complex_thread_local.h',
  ],
  deps = [
    '//third_party/aos/common:once',
    '//third_party/aos/common:die',
    '//linux_libraries:libpthread',
  ],
)

cc_test(
  name = 'complex_thread_local_test',
  srcs = [
    'complex_thread_local_test.cc',
  ],
  deps = [
    ':complex_thread_local',
    '//third_party/aos/testing:googletest',
    '//third_party/aos/common/util:thread',
  ],
)

cc_library(
  name = 'init',
  srcs = [
    'init.cc',
  ],
  hdrs = [
    'init.h',
  ],
  deps = [
    '//third_party/aos/common:die',
    '//third_party/aos/common:check',
    '//linux_libraries:libpthread',
  ],
)

cc_binary(
  name = 'core',
  srcs = [
    'core.cc',
  ],
  deps = [
    ':init',
    '//third_party/aos/common/util:run_command',
  ],
)

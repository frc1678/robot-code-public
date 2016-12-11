package(default_visibility = ['//visibility:public'])
licenses(['notice'])

py_library(
  name = 'py_trapezoid_profile',
  srcs = [
    'trapezoid_profile.py',
  ],
)

cc_library(
  name = 'run_command',
  srcs = [
    'run_command.cc',
  ],
  hdrs = [
    'run_command.h',
  ],
  deps = [
    '//third_party/aos/common:die',
    '//third_party/aos/common:check',
  ],
)

cc_test(
  name = 'run_command_test',
  srcs = [
    'run_command_test.cc',
  ],
  deps = [
    ':run_command',
    '//third_party/aos/testing:googletest',
    ':thread',
  ],
)

cc_library(
  name = 'inet_addr',
  srcs = [
    'inet_addr.cc',
  ],
  hdrs = [
    'inet_addr.h',
  ],
  deps = [
    '//third_party/aos/common:byteorder',
    '//third_party/aos/common:network_port',
  ],
)

cc_library(
  name = 'phased_loop',
  srcs = [
    'phased_loop.cc',
  ],
  hdrs = [
    'phased_loop.h',
  ],
  deps = [
    '//third_party/aos/common:check',
    '//third_party/aos/common:time',
  ],
)

cc_library(
  name = 'string_to_num',
  hdrs = [
    'string_to_num.h',
  ],
)

cc_test(
  name = 'string_to_num_test',
  srcs = [
    'string_to_num_test.cc',
  ],
  deps = [
    ':string_to_num',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'thread',
  srcs = [
    'thread.cc',
  ],
  hdrs = [
    'thread.h',
  ],
  deps = [
    '//third_party/aos/common:macros',
    '//third_party/aos/common:die',
    '//third_party/aos/common:check',
  ],
)

cc_library(
  name = 'trapezoid_profile',
  srcs = [
    'trapezoid_profile.cc',
  ],
  hdrs = [
    'trapezoid_profile.h',
  ],
  deps = [
    '//third_party/eigen',
    '//third_party/aos/common:time',
    # '//aos/common/logging',
    '//linux_libraries:libm',
  ],
)

cc_test(
  name = 'trapezoid_profile_test',
  srcs = [
    'trapezoid_profile_test.cc',
  ],
  deps = [
    ':trapezoid_profile',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'wrapping_counter',
  srcs = [
    'wrapping_counter.cc',
  ],
  hdrs = [
    'wrapping_counter.h',
  ],
)

cc_test(
  name = 'wrapping_counter_test',
  srcs = [
    'wrapping_counter_test.cc',
  ],
  deps = [
    ':wrapping_counter',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'options',
  hdrs = [
    'options.h',
  ],
)

cc_test(
  name = 'options_test',
  srcs = [
    'options_test.cc',
  ],
  deps = [
    ':options',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'compiler_memory_barrier',
  hdrs = [
    'compiler_memory_barrier.h',
  ],
)

cc_test(
  name = 'phased_loop_test',
  srcs = [
    'phased_loop_test.cc',
  ],
  deps = [
    ':phased_loop',
    '//third_party/aos/testing:googletest',
    '//linux_libraries:libpthread',
  ],
)

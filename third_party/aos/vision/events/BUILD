licenses(['notice'])

cc_library(
  name = 'epoll_events',
  srcs = ['epoll_events.cc'],
  hdrs = ['epoll_events.h'],
  deps = [
    '//third_party/aos/common:scoped_fd',
    '//third_party/aos/common:time',
  ],
)

cc_library(
  name = 'udp',
  visibility = ['//visibility:public'],
  srcs = ['udp.cc'],
  hdrs = ['udp.h'],
  deps = [
    '//third_party/aos/common:macros',
    '//third_party/aos/common:scoped_fd',
  ],
)

cc_test(
  name = 'udp_test',
  srcs = ['udp_test.cc'],
  deps = [
    ':udp',
    '//third_party/google/googletest:googletest'
  ],
)

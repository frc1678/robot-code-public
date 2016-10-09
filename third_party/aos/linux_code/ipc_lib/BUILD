package(default_visibility = ['//visibility:public'])
licenses(['notice'])

cc_library(
  name = 'aos_sync',
  srcs = [
    'aos_sync.cc',
  ],
  hdrs = [
    'aos_sync.h',
  ],
  deps = [
    '//third_party/aos/common:once',
    '//third_party/aos/common:macros',
    '//third_party/aos/common/util:compiler_memory_barrier',
    '//third_party/aos/common:die',
  ],
)

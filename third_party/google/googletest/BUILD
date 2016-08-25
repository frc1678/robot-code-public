licenses(['permissive'])

cc_library(
  name = 'googlemock',
  srcs = glob(['googlemock/src/*.cc'], ['googlemock/src/gmock-all.cc']),
  hdrs = glob([
    'googlemock/include/**/*.h'
  ]),
  includes = [
    'googlemock/include',
    'googlemock'
  ],
  deps = [':googletest'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'googletest',
  srcs = glob([
    'googletest/src/*',
    'googletest/include/gtest/internal/**/*.h'
  ], ['googletest/src/gtest-all.cc']),
  hdrs = glob(['googletest/include/gtest/*.h']),
  includes = [
    'googletest/include',
    'googletest'
  ],
  linkopts = ['-lpthread'],
  visibility = ['//visibility:public']
)

package(default_visibility = ['//visibility:public'])
load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'logger',
  srcs = ['logger.hpp'],
  hdrs = ['logger.h'],
  deps = ['filewriter',
          '//muan/queues:queues',
          '//muan/utils:proto_utils',
          '//muan/multithreading:updateable',
          '//third_party/google/protobuf',
          '//third_party/optional']
)

cc_test(
  name = 'logger_test',
  srcs = ['logger_test.cpp'],
  deps = ['//third_party/google/googletest',
          '//third_party/google/googletest:googlemock',
          'logger',
          'filewriter',
          'logger_test_protobufs',
          '//muan/utils:timing']
)

cc_proto_library(
  name = 'logger_test_protobufs',
  srcs = ['logger_test.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_library(
  name = 'filewriter',
  srcs = ['filewriter.hpp'],
  hdrs = ['filewriter.h'],
  deps = ['//third_party/boost:filesystem']
)

load('/third_party/google/protobuf/protobuf', 'cc_proto_library')

cc_library(
    name = "magazine",
    srcs = ['magazine.cpp'],
    hdrs = ['magazine.h'],
    deps = [':magazine_queues']
)

cc_proto_library(
  name = 'magazine_proto',
  srcs = ['magazine.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_library(
  name = 'magazine_queues',
  hdrs = ['queue_types.h'],
  deps = [
    ':magazine_proto',
    '//muan/proto:stack_proto',
    '//muan/queues:queues'
  ],
  visibility = ['//visibility:public']
)

cc_test(
  name = 'magazine_test',
  srcs = ['magazine_test.cpp'],
  deps = [
    ':magazine',
    '//third_party/google/googletest:googletest',
  ]
)


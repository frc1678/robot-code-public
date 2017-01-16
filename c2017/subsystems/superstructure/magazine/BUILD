load('/third_party/google/protobuf/protobuf', 'cc_proto_library')

cc_library(
	name = 'magazine_queues',
  hdrs = ['queue_types.h'],
  deps = ['magazine_proto', '//muan/proto:stack_proto', '//muan/queues:queues'],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'magazine_proto',
  srcs = ['magazine.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

    cc_library(name = 'ground_ball_intake', srcs = ['ground_ball_intake.cpp'],
               hdrs = ['ground_ball_intake.h'], deps = [
                 ':ground_ball_intake_queues',
                 '//muan/wpilib:wpilib_queues',
                 '//c2017/queue_manager:queue_manager'
               ],
               visibility = ['//visibility:public'])

        cc_library(name = 'ground_ball_intake_queues', hdrs = ['queue_types.h'],
                   deps = [ 'ground_ball_intake_proto', '//muan/proto:stack_proto', '//muan/queues:queues' ],
                   visibility = ['//visibility:public'])

            cc_proto_library(name = 'ground_ball_intake_proto', srcs = ['ground_ball_intake.proto'],
                             protoc = '//third_party/google/protobuf:protoc',
                             default_runtime = '//third_party/google/protobuf',
                             visibility = ['//visibility:public'])

                cc_test(name = 'ground_ball_intake_test', srcs = ['ground_ball_intake_test.cpp'], deps = [
                  ':ground_ball_intake_queues',
                  '//third_party/google/googletest:googletest',
                  '//muan/wpilib:wpilib_queues',
                  ':ground_ball_intake'
                ],
                        visibility = ['//visibility:public'])

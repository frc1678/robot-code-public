  #include "muan/proto/stack_proto.h"
  #include "muan/queues/message_queue.h"
  #include "c2017/subsystems/superstructure/intake/intake.pb.h"

  namespace c2017 {

  namespace ball_intake_proto {
    using IntakeOutputProto = muan::proto::StackProto<IntakeOutput, 1024>;
    using IntakeStatusProto = muan::proto::StackProto<IntakeStatus, 1024>;
    using IntakeGoalProto = muan::proto::StackProto<IntakeGoal, 1024>;
    using IntakeOutputQueue = muan::queues::MessageQueue<IntakeOutputProto, 100>;
    using IntakeStatusQueue = muan::queues::MessageQueue<IntakeStatusProto, 100>;
    using IntakeGoalQueue = muan::queues::MessageQueue<IntakeGoalProto, 100>;
    }
  }
  
  #endif

#ifndef C2017_SUBSYSTEMS_MAGAZINE_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_MAGAZINE_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/magazine.h"

namespace c2017 {
    
    namespace magazine {
        
        using MagazineGoalProto = muan::proto::StackProto<::c2017::subsystems::MagazineGoal, 256>;
        using MagazineInputProto = muan::proto::StackProto<::c2017::subsystems::MagazineInput, 256>;
        using MagazineOutputProto = muan::proto::StackProto<::c2017::subsystems::MagazineOutput, 256>;
        using MagazineStatusProto = muan::proto::StackProto<::c2017::subsystems::MagazineStatus, 256>;
        
        
        using MagazineGoalQueue = muan::queues::MessageQueue<MagazineGoalProto, 200>;
        using MagazineInputQueue = muan::queues::MessageQueue<MagazineInputProto, 200>;
        using MagazineOutputQueue = muan::queues::MessageQueue<MagazineOutputProto, 200>;
        using MagazineStatusQueue = muan::queues::MessageQueue<MagazineStatusProto, 200>;
    }
    
}

#endif

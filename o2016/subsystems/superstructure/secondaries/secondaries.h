#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_SECONDARIES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_SECONDARIES_H_

#include "queue_types.h"
#include "muan/proto/stack_proto.h"

namespace o2016 {
namespace secondaries {

class Secondaries {
  public:
    Secondaries() = default;
    SecondariesOutputProto Update(SecondariesGoalProto goal);
};

} // secondaries
} // o2016

#endif

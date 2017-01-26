#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_SECONDARIES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_SECONDARIES_H_

#include "muan/proto/stack_proto.h"
#include "o2016/subsystems/superstructure/secondaries/queue_types.h"

namespace o2016 {
namespace secondaries {

class Secondaries {
 public:
  Secondaries() = default;
  SecondariesOutputProto Update(SecondariesGoalProto goal);
};

}  // namespace secondaries
}  // namespace o2016

#endif  // O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_SECONDARIES_H_

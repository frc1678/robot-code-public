#ifndef O2018_AUTONOMOUS_NONE_H_
#define O2018_AUTONOMOUS_NONE_H_

#include "muan/logging/logger.h"
#include "o2018/autonomous/autonomous_base.h"

namespace o2018 {
namespace autonomous {

class None : public o2018::autonomous::AutonomousBase {
 public:
  void NoneAuto();
};

}  // namespace autonomous
}  // namespace o2018

#endif  // O2018_AUTONOMOUS_NONE_H_

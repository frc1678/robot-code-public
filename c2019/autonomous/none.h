#ifndef C2019_AUTONOMOUS_NONE_H_
#define C2019_AUTONOMOUS_NONE_H_

#include "muan/logging/logger.h"
#include "c2019/autonomous/autonomous_base.h"

namespace c2019 {
namespace autonomous {

class None : public c2019::autonomous::AutonomousBase {
 public:
  void NoneAuto();
};

}  // namespace autonomous
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_NONE_H_

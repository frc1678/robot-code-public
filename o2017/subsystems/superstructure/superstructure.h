#ifndef O2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define O2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "o2017/subsystems/superstructure/climber/climber.h"

namespace o2017 {

namespace superstructure {

class Superstructure {
 public:
  Superstructure();
  void Update();

 private:
  climber::Climber climber_;
};

}  // namespace superstructure

}  // namespace o2017

#endif  // O2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

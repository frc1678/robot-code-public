#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_

#include "queue_types.h"

namespace c2017 {

namespace magazine {

class Magazine {
 public:
  Magazine() = default;
  MagazineOutputProto Update(MagazineInputProto input);
  void SetGoal(MagazineGoalProto goal);

 private:
  bool has_hp_gear_;
  bool gear_intake_covered_;
  bool magazine_extended;
  double conveyor_voltage;
  double brush_voltage;

  bool score_gear_;
  bool rotate_gear_;

  c2017::magazine::ConveyorGoalState conveyor_goal_;
  c2017::magazine::HPIntakeGoalState hp_intake_goal_;
  c2017::magazine::BrushGoalState brush_goal_;
  c2017::magazine::MagazineOutputProto output_;
};

}  // magazine

}  // c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_

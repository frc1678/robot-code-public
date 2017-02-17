#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_

#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {

namespace magazine {

class Magazine {
 public:
  Magazine() = default;
  MagazineOutputProto Update(MagazineInputProto input, bool outputs_enabled);
  void SetGoal(MagazineGoalProto goal);

 private:
  bool has_hp_gear_;
  bool magazine_extended_;
  double score_gear_;
  c2017::magazine::HPIntakeGoalState hp_intake_goal_;
  c2017::magazine::UpperGoalState upper_goal_;
  c2017::magazine::LowerGoalState lower_goal_;
  c2017::magazine::SideGoalState side_goal_;

  MagazineStatusProto magazine_status_;
  MagazineOutputProto output_;
};

}  // namespace magazine

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_

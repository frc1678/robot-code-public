#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_ELEVATOR_ELEVATOR_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_ELEVATOR_ELEVATOR_H_

#include "c2019/subsystems/superstructure/elevator/queue_types.h"
#include "muan/units/units.h"
#include "muan/utils/disk_brake.h"
#include "muan/utils/math_utils.h"

namespace c2019 {
namespace elevator {

// Height constraints
constexpr double kMinHeight = 0.;
constexpr double kMaxHeight = 2.;
constexpr double kSecondStageHeight = 1.;

// Setting max voltage so that people don't die
constexpr double kMaxVoltage = 12.;

// Encoder faults
constexpr double kMinFaultVoltage = 4.;
constexpr double kMinFaultTicks = 400;

constexpr double kFF = 2.;
constexpr double kFFHatch = 0.5;
constexpr double kFFCargo = 0.3;
constexpr double kFFSecondStage = 1.;

class Elevator {
 public:
  Elevator();
  void Update(bool outputs_enabled, const ElevatorInputProto& input,
              ElevatorOutputProto* output, ElevatorStatusProto* status);
  void SetGoal(const ElevatorGoalProto& goal);

 private:
  double CalculateFeedForwards(bool has_panel, bool has_cargo,
                               bool second_stage);

  muan::DiskBrake disk_brake_{false};

  double height_goal_ = 0.;
  bool high_gear_ = false;
  bool wants_brake_ = false;
  bool crawling_ = false;
  bool crawler_down_ = false;

  double prev_encoder_ = 0.;
  int num_encoder_fault_ticks_ = 0;
  bool encoder_fault_ = false;
};

}  // namespace elevator
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_ELEVATOR_ELEVATOR_H_

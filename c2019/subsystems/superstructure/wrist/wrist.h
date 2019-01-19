#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_WRIST_WRIST_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_WRIST_WRIST_H_

#include <cmath>

#include "c2019/subsystems/superstructure/wrist/queue_types.h"
#include "muan/control/calibration/hall_calibration.h"

namespace c2019 {
namespace wrist {

// Angle constants
static constexpr double kMinAngle = 0;
static constexpr double kStowAngle =
    M_PI / 3;  // TODO(Apurva) find value for this
static constexpr double kMaxAngle = M_PI;

// Hall Calibration constants
static constexpr double kCalibVoltage = 0;
static constexpr double kHallEffectAngle =
    M_PI / 6;  // TODO(Apurva) find value for this

// Manual voltage control constants
static constexpr double kHoldingVoltage =
    5;  // TODO(Apurva) find value for this
static constexpr double kMaxVoltage = 12;

constexpr double kFF = 2.;  // TODO(alex) double check values
constexpr double kFFHatch = 0.5;
constexpr double kFFCargo = 0.3;

class Wrist {
 public:
  Wrist();

  void SetGoal(const WristGoalProto& goal);

  double CalculateFeedForwards(bool has_cargo, bool has_panel, double angle);

  // Updates the mode
  void Update(const WristInputProto& input, WristOutputProto* output,
              WristStatusProto* status, bool outputs_enabled);

  double TimeLeftUntil(double angle, double final_angle);

  // Getter for hall calibration
  inline bool is_calibrated() const {
    return hall_calibration_.is_calibrated();
  }

  inline double offset() const {
    return hall_calibration_.offset();
  }

 private:
  double goal_;

  muan::control::HallCalibration hall_calibration_{kHallEffectAngle};
};

}  // namespace wrist
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_WRIST_WRIST_H_

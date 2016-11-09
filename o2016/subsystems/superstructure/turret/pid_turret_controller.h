#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_TURRET_PIDTURRETCONTROLLER_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_TURRET_PIDTURRETCONTROLLER_H_

#include "queue_types.h"
#include "muan/control/pid_controller.h"
#include "muan/utils/offset_profiled_controller/pot_calibration.h"

namespace o2016 {

namespace turret {

class PidTurretController {
  public:
    PidTurretController();
    TurretOutputProto Update(const TurretInputProto& input);
    void SetGoal(const TurretGoalProto& goal);
  private:
    muan::PotCalibration calibration_;
    muan::PidController pid_;
    TurretGoalProto goal_;
    double current_position_ = 0;
};

}

}

#endif

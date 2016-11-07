#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_INTAKE_CONTROLLER_H_
#define O0216_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_INTAKE_CONTROLLER_H_

#include "o2016/subsystems/superstructure/intake/intake_constants.h"
#include "muan/units/units.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/utils/offset_profiled_controller/hall_calibration.h"


namespace o2016 {

namespace intake {

class IntakeController {
  public: 
    IntakeController();

    muan::units::Voltage Update(muan::units::Angle goal, muan::units::Angle sensor_input, bool index_click);
    muan::units::Angle GetAngle() const;
    void SetAngle(muan::units::Angle theta);
    bool AtGoal() const;
    bool is_calibrated() const;

  private:
    muan::control::StateSpaceController<1, 3, 1> controller_;
    muan::control::StateSpaceObserver<1, 3, 1> observer_;

    muan::HallCalibration calibration_;

    bool at_goal_;

    muan::units::Angle angle_tolerance_;
    muan::units::AngularVelocity velocity_tolerance_;
};


}

}



#endif 

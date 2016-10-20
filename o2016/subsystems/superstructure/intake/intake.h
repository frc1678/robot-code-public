#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_H_
#define O0216_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_H_

#include "o2016/subsystems/superstructure/intake/intake_constants.h"
#include "muan/units/units.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"

namespace frc1678{

namespace o2016 {

namespace intake {

class Intake {
  public: 
    Intake();
    ~Intake();
    muan::units::Voltage Update(muan::units::Angle goal, muan::units::Angle sensor_value);
    muan::units::Angle GetAngle() const;
    void SetAngle(muan::units::Angle theta);
    bool IsDone() const;

  private:
    muan::control::StateSpaceController<1, 2, 1> controller_;
    muan::control::StateSpaceObserver<1, 2, 1> observer_;
    bool done;

};

}

}

}



#endif 

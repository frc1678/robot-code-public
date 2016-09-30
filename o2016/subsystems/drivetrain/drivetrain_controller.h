#ifndef O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONTROLLER_H_
#define O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONTROLLER_H_

#include "Eigen/Core"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/units/units.h"
#include "o2016/subsystems/drivetrain/drivetrain.pb.h"

namespace frc1678 {

namespace drivetrain {

struct DrivetrainConstraints {
  muan::control::MotionProfileConstraints distance_constraints_;
  muan::control::MotionProfileConstraints angular_constraints_;

  // Provide a default constructor with garbage values just so we don't have to
  // deal with it in the constructor of DrivetrainController
  DrivetrainConstraints()
      : distance_constraints_{-1.0, -1.0}, angular_constraints_{-1.0, -1.0} {}
};

namespace controller {

class DrivetrainController {
 public:
  DrivetrainController();

  void Shift(Gear new_gear);

  DrivetrainOutput Update(const DrivetrainInput& input,
                          const DrivetrainGoal& goal);
  DrivetrainStatus GetStatus() const;

 private:
  muan::control::StateSpaceObserver<2, 7, 3> observer_;
  muan::control::StateSpaceController<2, 7, 3> controller_;

  DrivetrainConstraints GenerateTMPConstraints(
      muan::control::MotionProfilePosition initial_distance,
      muan::control::MotionProfilePosition final_distance,
      muan::control::MotionProfilePosition initial_angle,
      muan::control::MotionProfilePosition final_angle, Gear current_gear);

  muan::control::TrapezoidalMotionProfile distance_profile_;
  muan::control::TrapezoidalMotionProfile angle_profile_;

  DriveType drive_command_type_;
  Gear current_gear_;

  muan::units::Time elapsed_time_;
};

} /* controller */

} /* drivetrain */

} /* o2016 */

#endif /* ifndef O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONTROLLER_H_ */

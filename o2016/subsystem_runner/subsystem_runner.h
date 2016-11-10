#ifndef O2016_SUBSYSTEMRUNNER_SUBSYSTEMRUNNER_H_
#define O2016_SUBSYSTEMRUNNER_SUBSYSTEMRUNNER_H_

#include <atomic>

#include "o2016/queue_manager/queue_manager.h"
#include "o2016/subsystems/drivetrain/drivetrain_subsystem.h"
#include "o2016/subsystems/superstructure/intake/intake.h"
#include "o2016/subsystems/superstructure/intake/intake.h"
#include "o2016/subsystems/superstructure/secondaries/queue_types.h"
#include "o2016/subsystems/superstructure/catapult/catapult.h"
#include "o2016/subsystems/superstructure/catapult/queue_types.h"
#include "o2016/subsystems/superstructure/secondaries/secondaries.h"
#include "o2016/subsystems/superstructure/turret/pid_turret_controller.h"
#include "o2016/subsystems/superstructure/turret/queue_types.h"
#include "o2016/wpilib/wpilib_interface.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace o2016 {

class SubsystemRunner {
  public:
    SubsystemRunner();
    ~SubsystemRunner() = default;
    void operator()();
    void Stop();
  private:
    std::atomic<bool> running_;
    secondaries::Secondaries secondaries_;
    catapult::Catapult catapult_;
    catapult::CatapultInputQueue::QueueReader catapult_input_reader_;
    o2016::turret::PidTurretController turret_;
    o2016::turret::TurretInputQueue::QueueReader turret_input_ =
    QueueManager::GetInstance().turret_input_queue().MakeReader();
    o2016::drivetrain::DrivetrainSubsystem drivetrain_;
    intake::Intake intake_;
    wpilib::WpilibInterface wpilib_;
};
}
#endif

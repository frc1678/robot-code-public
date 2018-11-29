#ifndef O2018_INTERFACES_ARM_INTERFACE_H_
#define O2018_INTERFACES_ARM_INTERFACE_H_

#include "ctre/Phoenix.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/queue_types.h"

#include "o2018/subsystems/arm/queue_types.h"

namespace o2018 {
namespace interfaces {

using o2018::subsystems::arm::ArmInputProto;
using o2018::subsystems::arm::ArmInputQueue;
using o2018::subsystems::arm::ArmOutputProto;
using o2018::subsystems::arm::ArmOutputQueue;

constexpr uint32_t kArmId = 14;
constexpr uint32_t kIntakeLeft = 15;
constexpr uint32_t kIntakeRight = 16;

constexpr uint32_t kIntakeOpen = 2;
constexpr uint32_t kIntakeClose = 1;

class ArmInterface {
 public:
  ArmInterface();

  void ReadSensors();
  void WriteActuators();

 private:
  void LoadGains();
  void SetBrakeMode(bool mode);

  ArmInputQueue* input_queue_{
      muan::queues::QueueManager<ArmInputProto>::Fetch()};
  ArmOutputQueue::QueueReader output_reader_{
      muan::queues::QueueManager<ArmOutputProto>::Fetch()->MakeReader()};

  TalonSRX arm_talon_{kArmId};
  VictorSPX intake_left_{kIntakeLeft};
  TalonSRX intake_right_{kIntakeRight};

  DigitalInput intake_proxy_{0};
  DigitalInput arm_hall_{1};

  Solenoid intake_open_{kIntakeOpen};
  Solenoid intake_close_{kIntakeClose};
  bool has_been_zeroed_ = false;

  muan::wpilib::DriverStationQueue::QueueReader ds_reader_{
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch()
          ->MakeReader()};
};

}  // namespace interfaces
}  // namespace o2018

#endif  // O2018_INTERFACES_ARM_INTERFACE_H_

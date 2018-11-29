#ifndef O2018_INTERFACES_DRIVE_INTERFACE_H_
#define O2018_INTERFACES_DRIVE_INTERFACE_H_

#include "muan/phoenix/talon_wrapper.h"
#include "muan/phoenix/victor_wrapper.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/queue_types.h"

namespace o2018 {
namespace interfaces {

using muan::subsystems::drivetrain::InputProto;
using muan::subsystems::drivetrain::InputQueue;
using muan::subsystems::drivetrain::OutputProto;
using muan::subsystems::drivetrain::OutputQueue;

using muan::phoenix::TalonWrapper;
using muan::phoenix::VictorWrapper;

constexpr uint32_t kLeftMaster = 1;
constexpr uint32_t kRightMaster = 2;

constexpr uint32_t kLeftSlaveA = 13;
constexpr uint32_t kLeftSlaveB = 7;

constexpr uint32_t kRightSlaveA = 10;
constexpr uint32_t kRightSlaveB = 8;

class DrivetrainInterface {
 public:
  DrivetrainInterface();

  void ReadSensors();
  void WriteActuators();

 private:
  void LoadGains();
  void SetBrakeMode(bool mode);

  InputQueue* input_queue_;
  OutputQueue::QueueReader output_reader_;

  TalonSRX left_master_{kLeftMaster};
  TalonSRX right_master_{kRightMaster};

  VictorSPX left_slave_a_{kLeftSlaveA};
  VictorSPX left_slave_b_{kLeftSlaveB};

  TalonSRX right_slave_a_{kRightSlaveA};
  TalonSRX right_slave_b_{kRightSlaveB};

  PigeonIMU pigeon_;
  double pigeon_offset_ = 0;

  Solenoid shifter_{0};

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;
};

}  // namespace interfaces
}  // namespace o2018

#endif  // O2018_INTERFACES_DRIVE_INTERFACE_H_

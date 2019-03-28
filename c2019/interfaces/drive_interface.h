#ifndef C2019_INTERFACES_DRIVE_INTERFACE_H_
#define C2019_INTERFACES_DRIVE_INTERFACE_H_

#include <WPILib.h>
#include "ctre/Phoenix.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

namespace c2019 {
namespace interfaces {

using muan::subsystems::drivetrain::InputProto;
using muan::subsystems::drivetrain::InputQueue;
using muan::subsystems::drivetrain::OutputProto;
using muan::subsystems::drivetrain::OutputQueue;

constexpr uint32_t kLeftMaster = 2;
constexpr uint32_t kRightMaster = 1;

constexpr uint32_t kLeftSlaveA = 5;
constexpr uint32_t kLeftSlaveB = 6;

constexpr uint32_t kRightSlaveA = 3;
constexpr uint32_t kRightSlaveB = 4;

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

  TalonSRX left_slave_a_{kLeftSlaveA};
  VictorSPX left_slave_b_{kLeftSlaveB};

  VictorSPX right_slave_a_{kRightSlaveA};
  VictorSPX right_slave_b_{kRightSlaveB};

  PigeonIMU pigeon_;
  double pigeon_offset_ = 0;
  Compressor compressor_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;
};

}  // namespace interfaces
}  // namespace c2019

#endif  // C2019_INTERFACES_DRIVE_INTERFACE_H_

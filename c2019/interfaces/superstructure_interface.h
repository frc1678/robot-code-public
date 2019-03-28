#ifndef C2019_INTERFACES_SUPERSTRUCTURE_INTERFACE_H_
#define C2019_INTERFACES_SUPERSTRUCTURE_INTERFACE_H_

#include <WPILib.h>
#include "c2019/subsystems/superstructure/queue_types.h"
#include "muan/wpilib/can_wrapper.h"
#include "ctre/Phoenix.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace interfaces {

using superstructure::SuperstructureInputProto;
using superstructure::SuperstructureInputQueue;
using superstructure::SuperstructureOutputProto;
using superstructure::SuperstructureOutputQueue;

constexpr uint32_t kWinchTwo = 7;
constexpr uint32_t kElevatorMaster = 8;
constexpr uint32_t kElevatorSlaveA = 9;
constexpr uint32_t kElevatorSlaveB = 10;
constexpr uint32_t kElevatorSlaveC = 11;
constexpr uint32_t kWrist = 12;
constexpr uint32_t kCargoIntake = 13;
constexpr uint32_t kCrawler = 14;
constexpr uint32_t kWinch = 15;

constexpr uint32_t kShifter = 0;
constexpr uint32_t kPins = 0;
constexpr uint32_t kBackplate = 2;
constexpr uint32_t kArrow = 3;
constexpr uint32_t kCrawlerOne = 4;
constexpr uint32_t kForkDrop = 5;
constexpr uint32_t kCargo = 6;
constexpr uint32_t kBrake = 7;

class SuperstructureInterface {
 public:
  explicit SuperstructureInterface(muan::wpilib::CanWrapper* can_wrapper);
  void WriteActuators();
  void ReadSensors();

 private:
  void SetBrakeMode(bool mode);
  void LoadGains();

  SuperstructureInputQueue* input_queue_;
  SuperstructureOutputQueue::QueueReader output_reader_;

  TalonSRX elevator_master_{kElevatorMaster};
  VictorSPX elevator_slave_a_{kElevatorSlaveA};
  VictorSPX elevator_slave_b_{kElevatorSlaveB};
  VictorSPX elevator_slave_c_{kElevatorSlaveC};

  VictorSPX crawler_{kCrawler};
  VictorSPX winch_{kWinch};
  VictorSPX winch_two_{kWinchTwo};

  TalonSRX wrist_{kWrist};
  VictorSPX cargo_intake_{kCargoIntake};

  CANifier canifier_{0};

  muan::wpilib::PcmWrapper* pcm_;

  bool elevator_zeroed_ = false;
  bool wrist_zeroed_ = false;
  double wrist_offset_ = 0.0;
};

}  // namespace interfaces
}  // namespace c2019

#endif  // C2019_INTERFACES_SUPERSTRUCTURE_INTERFACE_H_

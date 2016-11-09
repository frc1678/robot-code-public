#ifndef O2016_WPILIB_WPILIB_INTERFACE_H_
#define O2016_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/can_wrapper.h"
#include "o2016/subsystems/drivetrain/queue_types.h"
#include "o2016/subsystems/superstructure/catapult/queue_types.h"
#include "o2016/subsystems/superstructure/intake/queue_types.h"
#include "o2016/subsystems/superstructure/turret/queue_types.h"
#include "o2016/subsystems/superstructure/secondaries/queue_types.h"

namespace o2016 {

namespace wpilib {

class DrivetrainInterface {
 public:
  DrivetrainInterface(muan::wpilib::CanWrapper* can_wrapper);

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::PcmWrapper* pcm_;

  o2016::drivetrain::DrivetrainInputQueue& input_queue_;
  o2016::drivetrain::DrivetrainOutputQueue::QueueReader output_queue_;

  VictorSP motor_left_a_, motor_left_b_;
  VictorSP motor_right_a_, motor_right_b_;

  Encoder encoder_left_, encoder_right_;
};

class TurretInterface {
 public:
  TurretInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  o2016::turret::TurretInputQueue& input_queue_;
  o2016::turret::TurretOutputQueue::QueueReader output_queue_;

  VictorSP motor_;
  Encoder encoder_;
  AnalogPotentiometer potentiometer_;
  Counter index_;
  int last_index_ = 0;
};

class IntakeInterface {
 public:
  IntakeInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  o2016::intake::IntakeInputQueue& input_queue_;
  o2016::intake::IntakeOutputQueue::QueueReader output_queue_;

  VictorSP motor_pivot_, motor_roller_;
  Encoder encoder_;
  Counter index_;
  int last_index_ = 0;
};

class CatapultInterface {
 public:
  CatapultInterface(muan::wpilib::CanWrapper* can);

  void WriteActuators();
  void ReadSensors();

 private:
  o2016::catapult::CatapultInputQueue& input_queue_;
  o2016::catapult::CatapultOutputQueue::QueueReader output_queue_;

  muan::wpilib::PcmWrapper* pcm_;

  VictorSP hard_stop_motor_;
  AnalogPotentiometer hard_stop_pot_;

  VictorSP scoop_motor_;
  AnalogPotentiometer scoop_pot_;
};

class SecondariesInterface {
 public:
  SecondariesInterface(muan::wpilib::CanWrapper* can);

  void WriteActuators();
 private:
  o2016::secondaries::SecondariesOutputQueue::QueueReader output_queue_;
  muan::wpilib::PcmWrapper* pcm_;
  VictorSP secondaries_motor_;
};

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;

  DrivetrainInterface drivetrain_;
  TurretInterface turret_;
  IntakeInterface intake_;
  CatapultInterface catapult_;
  SecondariesInterface secondaries_;
};

}  // wpilib

}  // o2016

#endif  // O2016_WPILIB_WPILIB_INTERFACE_H_

#ifndef _CATAPULT_MESSAGES_H_
#define _CATAPULT_MESSAGES_H_

#include "muan/units/units.h"

struct CatapultOutput {
  Voltage scoop_output;
  Voltage stop_output;
  bool cylinder_extend;
  bool disc_brake_activate;
};

struct CatapultStatus {
  Angle scoop_angle;
  AngularVelocity scoop_angular_velocity;
  Angle stop_angle;
  AngularVelocity stop_angular_velocity;
  Angle scoop_goal;
  Angle stop_goal;
  bool cylinder_extended;
  bool disk_brake_locked;
  bool scoop_terminated;
  bool stop_terminated;
  bool can_shoot;
  bool calibrated;
  // Some of those may have been unnecessary
};

struct CatapultInput {
  Angle scoop_pot;
  Angle stop_encoder;
  Angle stop_pot;
  Length distance_to_target;
};

struct CatapultGoal {
  bool should_shoot;
  bool should_tuck;
};

#endif // _CATAPULT_MESSAGES_H_

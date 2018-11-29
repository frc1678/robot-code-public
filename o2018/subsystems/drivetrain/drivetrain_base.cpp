#include "o2018/subsystems/drivetrain/drivetrain_base.h"

namespace o2018 {
namespace subsystems {
namespace drivetrain {

constexpr double kStallTorque = 1.41;
constexpr double kStallCurrent = 89;
constexpr double kFreeSpeed = 5840 * 2 * M_PI / 60;
constexpr double kFreeCurrent = 3;

constexpr double kMass = 45.;
constexpr double kDistRadius = 0.45;
constexpr double kMoment = kMass * kDistRadius * kDistRadius;

constexpr double kForceStiction = 32.65;
constexpr double kForceFriction = 20;
constexpr double kAngularDrag = -12;

constexpr double kRobotRadius = 12.0225 * 0.0254 / 2.0;
constexpr double kWheelRadius = 4.0 * 0.0254 / 2.0;

constexpr double kHighGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0);
constexpr double kLowGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (34.0 / 50.0);

constexpr double kHighGearEfficiency = 0.75;
constexpr double kLowGearEfficiency = 0.8;

muan::subsystems::drivetrain::DrivetrainConfig GetDrivetrainConfig() {
  muan::control::DriveTransmission::Properties high_gear{
      .num_motors = 3,
      .motor_kt = kStallTorque / kStallCurrent,
      .motor_kv = (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
      .motor_resistance = 12 / kStallCurrent,
      .gear_ratio = kHighGearRatio,
      .efficiency = kHighGearEfficiency,
  };

  muan::control::DriveTransmission::Properties low_gear{
      .num_motors = 3,
      .motor_kt = kStallTorque / kStallCurrent,
      .motor_kv = (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
      .motor_resistance = 12 / kStallCurrent,
      .gear_ratio = kLowGearRatio,
      .efficiency = kLowGearEfficiency,
  };

  muan::control::DrivetrainModel::Properties model{
      .wheelbase_radius = kRobotRadius,
      .angular_drag = kAngularDrag,  // TUNE ME
      .mass = kMass,
      .moment_inertia = kMoment,
      .force_stiction = kForceStiction,  // TUNE ME
      .force_friction = kForceFriction,  // TUNE ME
      .wheel_radius = kWheelRadius,
  };

  return {
      .high_gear_wheel_non_linearity = 0.65,
      .low_gear_wheel_non_linearity = 0.5,
      .high_gear_sensitivity = 0.65,
      .low_gear_sensitivity = 0.65,
      .beta = 2.0,
      .zeta = 0.7,
      .dt = 0.01,

      .max_velocity = 3.4,
      .max_acceleration = 6.0,
      .max_centripetal_acceleration = M_PI / 2.0,

      .high_gear_properties = high_gear,
      .low_gear_properties = low_gear,
      .drive_properties = model,
  };
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace o2018

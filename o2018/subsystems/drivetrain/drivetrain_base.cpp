#include "o2018/subsystems/drivetrain/drivetrain_base.h"

namespace o2018 {
namespace subsystems {
namespace drivetrain {

constexpr double kRobotRadius = 0.3489513;
constexpr double kWheelRadius = 0.0508;

constexpr double kMaxVoltage = 12.0;
constexpr double kFreeSpeed = 4.34 / kWheelRadius;  // rad / s

constexpr double kDriveKv = kMaxVoltage / kFreeSpeed;  // V / (rad / s)
constexpr double kDriveKa = 0.012;                     // V / (rad / s^2)
constexpr double kDriveKs = 1.3;                       // V

constexpr double kMass = 63.;                                  // kg
constexpr double kDistRadius = 0.46;                            // m
constexpr double kMoment = kMass * kDistRadius * kDistRadius;  // kg * m^2

constexpr double kAngularDrag = 12.0;  // N*m / (rad / s)

muan::subsystems::drivetrain::DrivetrainConfig GetDrivetrainConfig() {
  muan::control::DriveTransmission::Properties high_gear{
      .speed_per_volt = 1.0 / kDriveKv,
      .torque_per_volt = kWheelRadius * kWheelRadius * kMass / (2.0 * kDriveKa),
      .friction_voltage = kDriveKs,
  };

  muan::control::DriveTransmission::Properties low_gear =
      high_gear;  // One-speed :D

  muan::control::DrivetrainModel::Properties model{
      .mass = kMass,
      .moment_inertia = kMoment,
      .angular_drag = kAngularDrag,  // TUNE ME
      .wheel_radius = kWheelRadius,
      .wheelbase_radius = kRobotRadius,
  };

  return {
      .high_gear_wheel_non_linearity = 0.65,
      .low_gear_wheel_non_linearity = 0.5,
      .high_gear_sensitivity = 0.65,
      .low_gear_sensitivity = 0.65,
      .beta = 2.0,
      .zeta = 0.7,
      .dt = 0.02,

      .max_velocity = 3.5,
      .max_acceleration = 4.0,
      .max_centripetal_acceleration = 2.2,

      .high_gear_properties = high_gear,
      .low_gear_properties = low_gear,
      .drive_properties = model,
  };
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace o2018

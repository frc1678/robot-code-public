#ifndef MUAN_CONTROL_DRIVETRAIN_MODEL_H_
#define MUAN_CONTROL_DRIVETRAIN_MODEL_H_

#include <Eigen/Core>

namespace muan {
namespace control {

enum class ShifterRequest { kAutoShift, kLowGear, kHighGear };

enum class DrivetrainControlMode { kNone, kFollowingPath, kOpenLoop };

class DriveTransmission {
 public:
  struct Properties {
    // Motor constants
    double speed_per_volt;
    double torque_per_volt;
    double friction_voltage;
  };

  explicit DriveTransmission(Properties properties);

  double FreeSpeedAtVoltage(double voltage) const;
  double CalculateTorque(double velocity, double voltage) const;
  double VoltageFromTorque(double velocity, double torque) const;

  // Used in matrices elsewhere
  inline double speed_per_volt() const { return speed_per_volt_; }
  inline double torque_per_volt() const { return torque_per_volt_; }
  inline double friction_voltage() const { return friction_voltage_; }

 private:
  double speed_per_volt_, torque_per_volt_, friction_voltage_;
};

struct Bounds {
  double min = 0.0, max = 0.0;
};

class DrivetrainModel {
 public:
  struct Properties {
    // Mass of the robot, kg
    double mass;

    // Moment of inertia around the center of mass, around the z-axis, kg*m^2.
    double moment_inertia;

    // Modelled as a constant for the total torque on the drivetrain (CCW)
    // generated by CCW motion in (N*m)/(rad/s). Torque = angular_drag*omega.
    // This constant should be negative.
    double angular_drag = 0;

    // The radius of the wheel in meters.
    double wheel_radius;

    // Effective wheelbase radius, meters. Slightly larger than actual due to
    // skid.
    double wheelbase_radius;
  };

  // single speed
  DrivetrainModel(DrivetrainModel::Properties properties,
                  DriveTransmission transmission);

  // shifting two-speed
  DrivetrainModel(Properties properties, DriveTransmission low,
                  DriveTransmission high);

  // Left/right to linear/angular (wheel)
  Eigen::Vector2d ForwardKinematics(Eigen::Vector2d left_right) const;

  // Linear/angular to left/right (wheel)
  Eigen::Vector2d InverseKinematics(Eigen::Vector2d angular_linear) const;

  // Linear/angular velocity and left/right voltage to linear/angular
  // acceleration
  Eigen::Vector2d ForwardDynamics(Eigen::Vector2d velocity,
                                  Eigen::Vector2d voltage,
                                  bool high_gear = true) const;

  // Linear/angular velocity and linear/angular acceleration to left/right
  // voltage
  Eigen::Vector2d InverseDynamics(Eigen::Vector2d velocity,
                                  Eigen::Vector2d acceleration,
                                  bool high_gear = true) const;

  Eigen::Vector2d CalculateMaxVelocity(double curvature, double max_voltage, bool high_gear = true) const;

  // Calculate the minimum and maximum forwards (linear) acceleration along the
  // same curvature. `curvature` must be consistent with `velocity` if velocity
  // is nonzero.
  Bounds CalculateMinMaxAcceleration(Eigen::Vector2d velocity, double curvature,
                                     double max_voltage, bool high_gear = true) const;

 private:
  double wheelbase_radius_;
  double angular_drag_;
  double mass_;
  double moment_inertia_;
  double wheel_radius_;

  DriveTransmission transmission_low_, transmission_high_;
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_DRIVETRAIN_MODEL_H_

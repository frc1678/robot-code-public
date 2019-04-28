#include "muan/control/drivetrain_model.h"
#include <algorithm>
#include "muan/utils/math_utils.h"

namespace muan {
namespace control {

// Key:
// Chassis, means linear/angular m/s or rad/s
// Wheel, means left/right rad/s or N*m

DriveTransmission::DriveTransmission(Properties properties)
    : speed_per_volt_(properties.speed_per_volt),
      torque_per_volt_(properties.torque_per_volt),
      friction_voltage_(properties.friction_voltage) {}

double DriveTransmission::FreeSpeedAtVoltage(double voltage) const {
  double effective_voltage = 0.0;
  if (voltage > 1e-6) {
    // rolling forward, friciton negative
    effective_voltage = std::max(0.0, voltage - friction_voltage());
  } else if (voltage < -1e-6) {
    // rolling backward, friciton positive
    effective_voltage = std::min(0.0, voltage + friction_voltage());
  }

  return effective_voltage * speed_per_volt();
}

double DriveTransmission::CalculateTorque(double velocity,
                                          double voltage) const {
  double effective_voltage = voltage;
  if (velocity > 1e-6) {
    // rolling forward, friction negative
    effective_voltage -= friction_voltage();
  } else if (velocity < -1e-6) {
    // rolling backward, friction positive
    effective_voltage += friction_voltage();
  } else if (voltage > 1e-6) {
    // static, with positive voltage
    effective_voltage = std::max(0.0, voltage - friction_voltage());
  } else if (voltage < -1e-6) {
    // static, with negative voltage
    effective_voltage = std::min(0.0, voltage + friction_voltage());
  } else {
    return 0.0;
  }

  return torque_per_volt() * (effective_voltage - (velocity / speed_per_volt()));
}

double DriveTransmission::VoltageFromTorque(double velocity,
                                            double torque) const {
  double effective_friction_voltage;
  if (velocity > 1e-6) {
    // rolling forward, friction negative
    effective_friction_voltage = friction_voltage();
  } else if (velocity < -1e-6) {
    // rolling backward, friction positive
    effective_friction_voltage = -friction_voltage();
  } else if (torque > 1e-6) {
    // static, wants net torque forward
    effective_friction_voltage = friction_voltage();
  } else if (torque < -1e-6) {
    // static, wants net torque backward
    effective_friction_voltage = -friction_voltage();
  } else {
    return 0.0;
  }

  return torque / torque_per_volt() + velocity / speed_per_volt() +
         effective_friction_voltage;
}

DrivetrainModel::DrivetrainModel(DrivetrainModel::Properties properties,
                                 DriveTransmission transmission)
    : wheelbase_radius_(properties.wheelbase_radius),
      angular_drag_(properties.angular_drag),
      mass_(properties.mass),
      moment_inertia_(properties.moment_inertia),
      wheel_radius_(properties.wheel_radius),
      transmission_low_(transmission),
      transmission_high_(transmission) {}

DrivetrainModel::DrivetrainModel(DrivetrainModel::Properties properties,
                                 DriveTransmission low, DriveTransmission high)
    : wheelbase_radius_(properties.wheelbase_radius),
      angular_drag_(properties.angular_drag),
      mass_(properties.mass),
      moment_inertia_(properties.moment_inertia),
      wheel_radius_(properties.wheel_radius),
      transmission_low_(low),
      transmission_high_(high) {}

Eigen::Vector2d DrivetrainModel::ForwardKinematics(
    Eigen::Vector2d left_right) const {
  Eigen::Vector2d linear_angular;
  linear_angular(0) = (left_right(0) + left_right(1)) / 2.0;
  linear_angular(1) = (left_right(1) - left_right(0)) / (2 * wheelbase_radius_);
  return linear_angular * wheel_radius_;
}

Eigen::Vector2d DrivetrainModel::InverseKinematics(
    Eigen::Vector2d linear_angular) const {
  Eigen::Vector2d left_right;
  left_right(0) = (linear_angular(0) - linear_angular(1) * wheelbase_radius_);
  left_right(1) = (linear_angular(0) + linear_angular(1) * wheelbase_radius_);
  return left_right / wheel_radius_;
}

Eigen::Vector2d DrivetrainModel::ForwardDynamics(Eigen::Vector2d velocity,
                                                 Eigen::Vector2d voltage,
                                                 bool high_gear) const {
  DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;
  Eigen::Vector2d wheel_velocity = InverseKinematics(velocity);  // left, right

  // not moving, and not giving enough torque to start moving
  bool left_stuck = std::abs(wheel_velocity(0)) < 1e-6 &&
                    std::abs(voltage(0)) < transmission.friction_voltage();
  bool right_stuck = std::abs(wheel_velocity(1)) < 1e-6 &&
                     std::abs(voltage(1)) < transmission.friction_voltage();
  if (left_stuck && right_stuck) {
    // no movement
    return Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d wheel_torque;  // left, right
  wheel_torque(0) = transmission.CalculateTorque(wheel_velocity(0), voltage(0));
  wheel_torque(1) = transmission.CalculateTorque(wheel_velocity(1), voltage(1));

  Eigen::Vector2d chassis_accel;  // linear, angular
  chassis_accel(0) =
      (wheel_torque(0) + wheel_torque(1)) / (wheel_radius_ * mass_);
  chassis_accel(1) = wheelbase_radius_ * (wheel_torque(1) - wheel_torque(0)) /
                         (wheel_radius_ * moment_inertia_) -
                     (velocity(1) * angular_drag_ / moment_inertia_);

  return chassis_accel;
}

Eigen::Vector2d DrivetrainModel::InverseDynamics(Eigen::Vector2d velocity,
                                                 Eigen::Vector2d acceleration,
                                                 bool high_gear) const {
  DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;
  Eigen::Vector2d wheel_velocity = InverseKinematics(velocity);  // left, right

  Eigen::Vector2d wheel_torque;  // left, right
  wheel_torque(0) = 0.5 * wheel_radius_ *
                    (acceleration(0) * mass_ -
                     acceleration(1) * moment_inertia_ / wheelbase_radius_ -
                     velocity(1) * angular_drag_ / wheelbase_radius_);
  wheel_torque(1) = 0.5 * wheel_radius_ *
                    (acceleration(0) * mass_ +
                     acceleration(1) * moment_inertia_ / wheelbase_radius_ +
                     velocity(1) * angular_drag_ / wheelbase_radius_);

  Eigen::Vector2d wheel_voltage;  // left, right
  wheel_voltage(0) =
      transmission.VoltageFromTorque(wheel_velocity(0), wheel_torque(0));
  wheel_voltage(1) =
      transmission.VoltageFromTorque(wheel_velocity(1), wheel_torque(1));

  return wheel_voltage;
}

Eigen::Vector2d DrivetrainModel::CalculateMaxVelocity(double curvature,
                                                      double max_voltage,
                                                      bool high_gear) const {
  DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;
  double free_speed = transmission.FreeSpeedAtVoltage(max_voltage);
  if (std::abs(curvature) < 1e-6) {
    return ForwardKinematics(
        Eigen::Vector2d(free_speed, free_speed).cwiseAbs());
  }

  double right_constrained = free_speed *
                             (wheelbase_radius_ * curvature + 1.0) /
                             (1.0 - wheelbase_radius_ * curvature);
  if (std::abs(right_constrained) <= free_speed) {
    return ForwardKinematics(
        Eigen::Vector2d(free_speed, right_constrained).cwiseAbs());
  }

  double left_constrained = free_speed * (1.0 - wheelbase_radius_ * curvature) /
                            (1.0 + wheelbase_radius_ * curvature);

  return ForwardKinematics(
      Eigen::Vector2d(left_constrained, free_speed).cwiseAbs());
}

Bounds DrivetrainModel::CalculateMinMaxAcceleration(Eigen::Vector2d velocity,
                                                    double curvature,
                                                    double max_voltage,
                                                    bool high_gear) const {
  DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;
  Bounds result{.min = 1e9, .max = -1e9};

  Eigen::Vector2d wheel_velocity = InverseKinematics(velocity);  // left, right

  const double linear_term =
      (std::abs(velocity(0)) < 1e-4) ? 0.0 : mass_ * wheelbase_radius_;
  const double angular_term = (std::abs(velocity(0)) < 1e-4)
                                  ? moment_inertia_
                                  : moment_inertia_ * curvature;

  const double drag_torque = velocity(1) * angular_drag_;

  std::array<bool, 2> left_right = {false, true};
  std::array<double, 2> positive_negative = {1.0, -1.0};

  for (bool left : left_right) {
    for (double sign : positive_negative) {
      const DriveTransmission fixed_transmission = transmission;
      const DriveTransmission variable_transmission = transmission;

      const double fixed_torque = fixed_transmission.CalculateTorque(
          wheel_velocity(static_cast<int>(!left)), sign * max_voltage);
      double variable_torque = 0.0;

      if (left) {
        variable_torque = (-drag_torque * mass_ * wheel_radius_ +
                           fixed_torque * (linear_term + angular_term)) /
                          (linear_term - angular_term);
      } else {
        variable_torque = (drag_torque * mass_ * wheel_radius_ +
                           fixed_torque * (linear_term - angular_term)) /
                          (linear_term + angular_term);
      }
      const double variable_voltage = variable_transmission.VoltageFromTorque(
          wheel_velocity(static_cast<int>(left)), variable_torque);
      if (std::abs(variable_voltage) <= max_voltage + 1e-6) {
        double accel = 0.0;
        if (std::abs(velocity(0)) < 1e-4) {
          accel = (left ? -1.0 : 1.0) * (fixed_torque - variable_torque) *
                      wheelbase_radius_ / (moment_inertia_ * wheel_radius_) -
                  drag_torque / moment_inertia_;
        } else {
          accel = (fixed_torque + variable_torque) / (mass_ * wheel_radius_);
        }

        result.min = std::copysign(std::min(result.min, accel), -1.0);
        result.max = std::copysign(std::max(result.max, accel), 1.0);
      }
    }
  }

  return result;
}

}  // namespace control
}  // namespace muan

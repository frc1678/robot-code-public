#include "third_party/frc971/control_loops/drivetrain/polydrivetrain.h"

#include "third_party/aos/common/commonmath.h"
#include "third_party/aos/common/controls/polytope.h"

#include "third_party/frc971/control_loops/coerce_goal.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

using ::frc971::control_loops::CoerceGoal;

PolyDrivetrain::PolyDrivetrain(const DrivetrainConfig &dt_config,
                               StateFeedbackLoop<7, 2, 3> *kf)
    : kf_(kf),
      U_Poly_((Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
               /*[*/ -1, 0 /*]*/,
               /*[*/ 0, 1 /*]*/,
               /*[*/ 0, -1 /*]]*/)
                  .finished(),
              (Eigen::Matrix<double, 4, 1>() << /*[[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]*/,
               /*[*/ 12 /*]]*/)
                  .finished()),
      loop_(new StateFeedbackLoop<2, 2, 2>(dt_config.make_v_drivetrain_loop())),
      ttrust_(1.1),
      wheel_(0.0),
      throttle_(0.0),
      quickturn_(false),
      left_gear_(dt_config.default_high_gear ? Gear::kHighGear
                                             : Gear::kLowGear),
      right_gear_(dt_config.default_high_gear ? Gear::kHighGear
                                              : Gear::kLowGear),
      counter_(0),
      dt_config_(dt_config) {
  last_position_->set_left_encoder(0.0);
  last_position_->set_right_encoder(0.0);
  position_->set_left_encoder(0.0);
  position_->set_right_encoder(0.0);
}

double PolyDrivetrain::MotorSpeed(double velocity, Gear gear) {
  const double high_gear_speed =
      velocity / dt_config_.high_gear_ratio / dt_config_.wheel_radius;
  const double low_gear_speed =
      velocity / dt_config_.low_gear_ratio / dt_config_.wheel_radius;
  // Not in gear, so speed-match to destination gear.
  switch (gear) {
    case Gear::kHighGear:
      return high_gear_speed;
    case Gear::kLowGear:
    default:
      return low_gear_speed;
      break;
  }
}

void PolyDrivetrain::SetGoal(
    const ::frc971::control_loops::drivetrain::GoalProto &goal) {
  // We should only deal with teleop goals here
  if (!goal->has_teleop_command()) {
    return;
  }

  auto teleop_goal = goal->teleop_command();

  const double wheel = teleop_goal.steering();
  const double throttle = teleop_goal.throttle();
  const bool quickturn = teleop_goal.quick_turn();
  const bool highgear = goal->gear() == Gear::kHighGear;

  const double kWheelNonLinearity = highgear ? 0.6 : 0.5;
  // Apply a sin function that's scaled to make it feel better.
  const double angular_range = M_PI_2 * kWheelNonLinearity;

  wheel_ = tan(angular_range * wheel) / tan(angular_range);
  wheel_ = tan(angular_range * wheel_) / tan(angular_range);
  wheel_ = 2.0 * wheel - wheel_;
  quickturn_ = quickturn;

  static const double kThrottleDeadband = 0.05;
  if (::std::abs(throttle) < kThrottleDeadband) {
    throttle_ = 0;
  } else {
    throttle_ = copysign(
        (::std::abs(throttle) - kThrottleDeadband) / (1.0 - kThrottleDeadband),
        throttle);
  }

  Gear requested_gear = highgear ? Gear::kHighGear : Gear::kLowGear;

  left_gear_ = requested_gear;
  right_gear_ = requested_gear;
}

void PolyDrivetrain::SetPosition(
    const ::frc971::control_loops::drivetrain::InputProto *position,
    Gear left_gear, Gear right_gear) {
  left_gear_ = left_gear;
  right_gear_ = right_gear;
  last_position_ = position_;
  position_ = *position;
}

double PolyDrivetrain::FilterVelocity(double throttle) const {
  const Eigen::Matrix<double, 2, 2> FF =
      loop_->B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

  constexpr int kHighGearController = 3;
  const Eigen::Matrix<double, 2, 2> FF_high =
      loop_->controller(kHighGearController).plant.B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() -
       loop_->controller(kHighGearController).plant.A());

  ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
  int min_FF_sum_index;
  const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
  const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
  const double high_min_FF_sum = FF_high.col(0).sum();

  const double adjusted_ff_voltage =
      ::aos::Clip(throttle * 12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
  return (adjusted_ff_voltage +
          ttrust_ * min_K_sum * (loop_->X_hat(0, 0) + loop_->X_hat(1, 0)) /
              2.0) /
         (ttrust_ * min_K_sum + min_FF_sum);
}

double PolyDrivetrain::MaxVelocity() {
  const Eigen::Matrix<double, 2, 2> FF =
      loop_->B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

  constexpr int kHighGearController = 3;
  const Eigen::Matrix<double, 2, 2> FF_high =
      loop_->controller(kHighGearController).plant.B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() -
       loop_->controller(kHighGearController).plant.A());

  ::Eigen::Matrix<double, 1, 2> FF_sum = FF.colwise().sum();
  int min_FF_sum_index;
  const double min_FF_sum = FF_sum.minCoeff(&min_FF_sum_index);
  // const double min_K_sum = loop_->K().col(min_FF_sum_index).sum();
  const double high_min_FF_sum = FF_high.col(0).sum();

  const double adjusted_ff_voltage =
      ::aos::Clip(12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0);
  return adjusted_ff_voltage / min_FF_sum;
}

void PolyDrivetrain::Update() {
  if (dt_config_.loop_type == LoopType::CLOSED_LOOP) {
    loop_->mutable_X_hat()(0, 0) = kf_->X_hat()(1, 0);
    loop_->mutable_X_hat()(1, 0) = kf_->X_hat()(3, 0);
    std::cout << loop_->mutable_X_hat() << std::endl;
  }

  // TODO(austin): Observer for the current velocity instead of difference
  // calculations.
  ++counter_;

  // FF * X = U (steady state)
  const Eigen::Matrix<double, 2, 2> FF =
      loop_->B().inverse() *
      (Eigen::Matrix<double, 2, 2>::Identity() - loop_->A());

  // Invert the plant to figure out how the velocity filter would have to
  // work
  // out in order to filter out the forwards negative inertia.
  // This math assumes that the left and right power and velocity are
  // equals,
  // and that the plant is the same on the left and right.
  const double fvel = FilterVelocity(throttle_);

  const double sign_svel = wheel_ * ((fvel > 0.0) ? 1.0 : -1.0);
  double steering_velocity;
  if (quickturn_) {
    steering_velocity = wheel_ * MaxVelocity();
  } else {
    steering_velocity = ::std::abs(fvel) * wheel_;
  }
  const double left_velocity = fvel - steering_velocity;
  const double right_velocity = fvel + steering_velocity;
  goal_left_velocity_ = left_velocity;
  goal_right_velocity_ = right_velocity;

  // Integrate velocity to get the position.
  // This position is used to get integral control.
  loop_->mutable_R() << left_velocity, right_velocity;

  if (!quickturn_) {
    // K * R = w
    Eigen::Matrix<double, 1, 2> equality_k;
    equality_k << 1 + sign_svel, -(1 - sign_svel);
    const double equality_w = 0.0;

    // Construct a constraint on R by manipulating the constraint on U
    ::aos::controls::HPolytope<2> R_poly = ::aos::controls::HPolytope<2>(
        U_Poly_.H() * (loop_->K() + FF),
        U_Poly_.k() + U_Poly_.H() * loop_->K() * loop_->X_hat());

    // Limit R back inside the box.
    loop_->mutable_R() = CoerceGoal(R_poly, equality_k, equality_w, loop_->R());
  }

  const Eigen::Matrix<double, 2, 1> FF_volts = FF * loop_->R();
  const Eigen::Matrix<double, 2, 1> U_ideal =
      loop_->K() * (loop_->R() - loop_->X_hat()) + FF_volts;

  for (int i = 0; i < 2; i++) {
    loop_->mutable_U()[i] = ::aos::Clip(U_ideal[i], -12, 12);
  }

  if (dt_config_.loop_type == LoopType::OPEN_LOOP) {
    loop_->mutable_X_hat() =
        loop_->A() * loop_->X_hat() + loop_->B() * loop_->U();
  }
}

void PolyDrivetrain::SetOutput(
    ::frc971::control_loops::drivetrain::OutputProto *output) {
  if (output != NULL) {
    (*output)->set_left_voltage(loop_->U(0, 0));
    (*output)->set_right_voltage(loop_->U(1, 0));
    (*output)->set_high_gear(left_gear_);
  }
}

void PolyDrivetrain::PopulateStatus(
    ::frc971::control_loops::drivetrain::StatusProto *status) {
  (*status)->set_left_velocity_goal(goal_left_velocity_);
  (*status)->set_right_velocity_goal(goal_right_velocity_);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

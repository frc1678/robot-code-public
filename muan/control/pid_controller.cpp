#include "muan/control/pid_controller.h"

namespace muan {

PidController::PidController(double kP, double kI, double kD)
    : kP(kP), kI(kI), kD(kD), integral_(0), last_proportional_(0) {}
PidController::PidController(const PidGains& gains)
    : PidController(gains.kP, gains.kI, gains.kD) {}

double PidController::Calculate(muan::units::Time dt, double error) {
  return ((error * kP) + (CalculateIntegral(dt, error) * kI) +
          (last_derivative_ = CalculateDerivative(dt, error)) * kD);
}

void PidController::SetProportionalConstant(double p) { kP = p; }

void PidController::SetIntegralConstant(double i) { kI = i; }

void PidController::SetDerivativeConstant(double d) { kD = d; }

void PidController::SetGains(const PidGains& gains) {
  kP = gains.kP;
  kI = gains.kI;
  kD = gains.kD;
}

void PidController::Reset() {
  last_proportional_ = 0;
  integral_ = 0;
}

double PidController::GetDerivative() { return last_derivative_; }

double PidController::CalculateDerivative(muan::units::Time dt, double error) {
  auto ret = (error - last_proportional_) / dt;
  last_proportional_ = error;
  return ret;
}

double PidController::CalculateIntegral(muan::units::Time dt, double error) {
  integral_ += error * dt;
  return integral_;
}

}  // namespace muan

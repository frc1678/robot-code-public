#ifndef MUAN_CONTROL_PID_CONTROLLER_H_
#define MUAN_CONTROL_PID_CONTROLLER_H_

#include "muan/units/units.h"
#include <type_traits>

namespace muan {

class PidController {
 public:
  using ProportionalConstant = double;
  using IntegralConstant = double;
  using DerivativeConstant = double;

  struct PidGains {
    ProportionalConstant kP;
    IntegralConstant kI;
    DerivativeConstant kD;
  };

  PidController(ProportionalConstant kP, IntegralConstant kI,
                DerivativeConstant kD)
      : kP(kP), kI(kI), kD(kD), integral_(0), last_proportional_(0) {}
  PidController(const PidGains& gains)
      : PidController(gains.kP, gains.kI, gains.kD) {}

  double Calculate(muan::units::Seconds dt, double error) {
    return error * kP + CalculateIntegral(dt, error) * kI +
           (last_derivative_ = CalculateDerivative(dt, error)) * kD;
  }

  void SetProportionalConstant(ProportionalConstant p) { kP = p; }

  void SetIntegralConstant(IntegralConstant i) { kI = i; }

  void SetDerivativeConstant(DerivativeConstant d) { kD = d; }

  void SetGains(const PidGains& gains) {
    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;
  }

  void Reset() {
    last_proportional_ = 0;
    integral_ = 0;
  }

  double GetDerivative() { return last_derivative_; }

 protected:
  double CalculateDerivative(muan::units::Seconds dt, double error) {
    auto ret = (error - last_proportional_) / dt;
    last_proportional_ = error;
    return ret;
  }

  double CalculateIntegral(muan::units::Seconds dt, double error) {
    integral_ += error * dt;
    return integral_;
  }

  ProportionalConstant kP;
  IntegralConstant kI;
  DerivativeConstant kD;
  double integral_;
  double last_proportional_;
  double last_derivative_;
};
}

#endif

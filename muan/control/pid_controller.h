#ifndef MUAN_CONTROL_PID_CONTROLLER_H_
#define MUAN_CONTROL_PID_CONTROLLER_H_

#include "muan/units/units.h"
#include <type_traits>

namespace muan {

class PidController {
 public:

  struct PidGains {
    double kP;
    double kI;
    double kD;
  };

  PidController(double kP, double kI,
                double kD)
      : kP(kP), kI(kI), kD(kD), integral_(0), last_proportional_(0) {}
  PidController(const PidGains& gains)
      : PidController(gains.kP, gains.kI, gains.kD) {}

  double Calculate(muan::units::Time dt, double error) {
    return error * kP + CalculateIntegral(dt, error) * kI +
           (last_derivative_ = CalculateDerivative(dt, error)) * kD;
  }

  void SetProportionalConstant(double p) { kP = p; }

  void SetDerivativeConstant(double i) { kI = i; }

  void Setdouble(double d) { kD = d; }

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
  double CalculateDerivative(muan::units::Time dt, double error) {
    auto ret = (error - last_proportional_) / dt;
    last_proportional_ = error;
    return ret;
  }

  double CalculateIntegral(muan::units::Time dt, double error) {
    integral_ += error * dt;
    return integral_;
  }

  double kP;
  double kI;
  double kD;
  double integral_;
  double last_proportional_;
  double last_derivative_;
};
}

#endif

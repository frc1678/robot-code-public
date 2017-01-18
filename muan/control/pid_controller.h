#ifndef MUAN_CONTROL_PID_CONTROLLER_H_
#define MUAN_CONTROL_PID_CONTROLLER_H_

#include <type_traits>
#include "muan/units/units.h"

namespace muan {

class PidController {
 public:
  struct PidGains {
    double kP;
    double kI;
    double kD;
  };

  PidController(double kP, double kI, double kD);
  explicit PidController(const PidGains& gains);

  double Calculate(muan::units::Time dt, double error);

  void SetProportionalConstant(double p);

  void SetDerivativeConstant(double i);

  void Setdouble(double d);

  void SetGains(const PidGains& gains);

  void Reset();

  double GetDerivative();

 protected:
  double CalculateDerivative(muan::units::Time dt, double error);

  double CalculateIntegral(muan::units::Time dt, double error);

  double kP;
  double kI;
  double kD;
  double integral_;
  double last_proportional_;
  double last_derivative_;
};

}  // namespace muan

#endif

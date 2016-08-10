#ifndef MUAN_CONTROL_AVERAGE_FILTER_PID_H_
#define MUAN_CONTROL_AVERAGE_FILTER_PID_H_

#include "pid_controller.h"
#include "utils/history.h"

namespace muan {

template <typename InputType, typename OutputType, int HistLength = 5>
class AverageFilterPidController : public PidController<InputType, OutputType> {
 public:
  using ProportionalConstant =
      typename std::remove_cv<decltype(OutputType(0) / InputType(0))>::type;
  using IntegralConstant = typename std::remove_cv<decltype(
      ProportionalConstant(0) / Time(0))>::type;
  using DerivativeConstant = typename std::remove_cv<decltype(
      ProportionalConstant(0) * Time(0))>::type;

  using PidGains = typename PidController<InputType, OutputType>::PidGains;

  AverageFilterPidController(ProportionalConstant kP, IntegralConstant kI,
                             DerivativeConstant kD)
      : PidController<InputType, OutputType>(kP, kI, kD), hist_(.005 * s) {}
  AverageFilterPidController(const PidGains& gains)
      : PidController<InputType, OutputType>(gains), hist_(.005 * s) {}

 protected:
  decltype(InputType(0) / s) CalculateDerivative(Time dt, InputType error) {
    auto current_derivative_ =
        PidController<InputType, OutputType>::CalculateDerivative(dt, error);
    auto total = InputType(0) / s;
    for (auto d : hist_) {
      total += d;
    }
    return total / HistLength;
  }

  History<decltype(InputType(0) / s), HistLength> hist_;
};
}

#endif

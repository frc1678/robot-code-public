#ifndef MUAN_CONTROL_AVERAGE_FILTER_PID_H_
#define MUAN_CONTROL_AVERAGE_FILTER_PID_H_

#include "pid_controller.h"
#include "muan/utils/history.h"

namespace muan {

template <typename InputType, typename OutputType, int HistLength = 5>
class AverageFilterPidController : public PidController<InputType, OutputType> {
 public:
  using ProportionalConstant =
      Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2,
            OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4>;
  using IntegralConstant =
      Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2 - 1,
            OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4>;
  using DerivativeConstant =
      Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2 + 1,
            OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4>;
  AverageFilterPidController(ProportionalConstant kP, IntegralConstant kI,
                             DerivativeConstant kD)
      : PidController<InputType, OutputType>(kP, kI, kD), hist_(.005 * s) {}

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

#ifndef MUAN_CONTROL_STATE_FEEDBACK_CONTROLLER_H_
#define MUAN_CONTROL_STATE_FEEDBACK_CONTROLLER_H_

#include "Eigen/Core"
#include "muan/unitscpp/unitscpp.h"
#include <iostream>

namespace muan {

template <int States, int Outputs>
class StateFeedbackController {
 public:
  StateFeedbackController() {}
  StateFeedbackController(Eigen::Matrix<double, Outputs, States> k) : k_(k) {
    for (int i = 0; i < States; i++) {
      goal_(i) = 0;
    }
  }
  Eigen::Matrix<double, Outputs, 1> Calculate(
      Eigen::Matrix<double, States, 1> state) {
    return k_ * (goal_ - state);
  }
  void SetGains(Eigen::Matrix<double, Outputs, States> k) { k_ = k; }

  void SetGoal(Eigen::Matrix<double, States, 1> goal) { goal_ = goal; }

 private:
  Eigen::Matrix<double, Outputs, States> k_;
  Eigen::Matrix<double, States, 1> goal_;
};
}

#endif

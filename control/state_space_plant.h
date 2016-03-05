#ifndef MUAN_CONTROL_STATE_SPACE_PLANT_H_
#define MUAN_CONTROL_STATE_SPACE_PLANT_H_

#include "Eigen/Core"
#include "muan/unitscpp/unitscpp.h"
#include <cmath>

namespace muan {

template <int Inputs, int States, int Outputs, bool discrete = false>
class StateSpacePlant {
 public:
  StateSpacePlant(Time dt) : dt_(dt) {}
  StateSpacePlant(Eigen::Matrix<double, States, States> a,
                  Eigen::Matrix<double, States, Outputs> b,
                  Eigen::Matrix<double, Inputs, States> c, Time dt = 0 * s)
      : a_(a), b_(b), c_(c), dt_(dt) {
    for (int i = 0; i < States; i++) {
      x_(i) = 0;
    }
  }

  void Update(const Eigen::Matrix<double, Outputs, 1>& u) {
    static_assert(
        discrete,
        "You have to supply a delta time parameter to a continuous system!");
    x_ = a_ * x_ + b_ * u;
  }

  void Update(const Eigen::Matrix<double, Outputs, 1>& u, Time dt) {
    static_assert(!discrete, "Don't supply a delta time to a discrete system!");
    x_ += (a_ * x_ + b_ * u) * dt.to(s);
  }

  Eigen::Matrix<double, States, 1> GetX() { return x_; }

  void SetX(const Eigen::Matrix<double, States, 1>& x) { x_ = x; }

  void SetSystem(Eigen::Matrix<double, States, States> A,
                 Eigen::Matrix<double, States, Outputs> B,
                 Eigen::Matrix<double, Inputs, States> C) {
    a_ = A;
    b_ = B;
    c_ = C;
  }

  Eigen::Matrix<double, Inputs, 1> GetY() { return c_ * x_; }

  Eigen::Matrix<double, States, States> GetA() { return a_; }

  Eigen::Matrix<double, States, Outputs> GetB() { return b_; }

  Eigen::Matrix<double, Inputs, States> GetC() { return c_; }

  Time GetTimestep() {
    static_assert(discrete, "Only a discrete-time model has a fixed timestep");
    return dt_;
  }

 private:
  Eigen::Matrix<double, States, States> a_;
  Eigen::Matrix<double, States, Outputs> b_;
  Eigen::Matrix<double, Inputs, States> c_;

  Eigen::Matrix<double, States, 1> x_;
  Time dt_;
};
}

#endif

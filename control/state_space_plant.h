#ifndef MUAN_CONTROL_STATE_SPACE_PLANT_H_
#define MUAN_CONTROL_STATE_SPACE_PLANT_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "unitscpp/unitscpp.h"
#include "unsupported/Eigen/MatrixFunctions"
#include <cmath>

namespace muan {

template <int Inputs, int States, int Outputs, bool discrete = false>
class StateSpacePlant {
 public:
  StateSpacePlant(Eigen::Matrix<double, States, States> a,
                  Eigen::Matrix<double, States, Outputs> b,
                  Eigen::Matrix<double, Inputs, States> c)
      : a_(a), b_(b), c_(c) {}

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

  Eigen::Matrix<double, Inputs, 1> GetY() { return c_ * x_; }

  Eigen::Matrix<double, States, States> GetA() {
    return a_;
  }

  Eigen::Matrix<double, States, Outputs> GetB() {
    return b_;
  }

  Eigen::Matrix<double, Inputs, States> GetC() {
    return c_;
  }

 private:
  Eigen::Matrix<double, States, States> a_;
  Eigen::Matrix<double, States, Outputs> b_;
  Eigen::Matrix<double, Inputs, States> c_;

  Eigen::Matrix<double, States, 1> x_;
};

/*
 * c2d: Converts a state-space system from continuous to discrete time with a
 * fixed timestep.
 * This function was ported from 971's implementation of the c2d function.
 */
template <int Inputs, int States, int Outputs>
StateSpacePlant<Inputs, States, Outputs, true> c2d(
    Eigen::Matrix<double, States, States> a,
    Eigen::Matrix<double, States, Outputs> b,
    Eigen::Matrix<double, Inputs, States> c, Time dt) {
  Eigen::EigenSolver<Eigen::Matrix<double, States, States>> solver(a);
  Eigen::Matrix<double, States, 1> evals = solver.eigenvalues().real();
  Eigen::Matrix<double, States, States> evecs = solver.eigenvectors().real();
  Eigen::Matrix<double, States, States> diag =
      Eigen::Matrix<std::complex<double>, States, States>::Identity().real();
  Eigen::Matrix<double, States, States> diage =
      Eigen::Matrix<std::complex<double>, States, States>::Identity().real();

  for (int i = 0; i < evals.rows(); i++) {
    diag(i, i) = std::exp(dt.to(s) * evals(i));
    if (std::abs(evals(i)) < 1e-16) {
      diage(i, i) = dt.to(s);
    } else {
      diage(i, i) = (std::exp(evals(i) * dt.to(s)) - 1.0) / evals(i);
    }
  }

  Eigen::Matrix<double, States, States> a_discrete =
      (evecs * diag * evecs.inverse());
  Eigen::Matrix<double, States, Outputs> b_discrete =
      (evecs * diage * evecs.inverse() * b);
  return StateSpacePlant<Inputs, States, Outputs, true>(a_discrete, b_discrete,
                                                        c);
}

template<int Inputs, int States, int Outputs>
StateSpacePlant<Inputs, States, Outputs, true> c2d(
                StateSpacePlant<Inputs, States, Outputs> sys, Time dt) {
  return c2d(sys.GetA(), sys.GetB(), sys.GetC(), dt);
}

}

#endif

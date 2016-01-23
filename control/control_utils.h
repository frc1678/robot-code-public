#include "Eigen/Eigenvalues"
#include "state_space_plant.h"
#include "state_feedback_controller.h"
#include <vector>

namespace muan {

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

template <int Inputs, int States, int Outputs>
StateSpacePlant<Inputs, States, Outputs, true> c2d(
    StateSpacePlant<Inputs, States, Outputs> sys, Time dt) {
  return c2d(sys.GetA(), sys.GetB(), sys.GetC(), dt);
}

template <int rows, int cols>
Eigen::Matrix<double, rows, cols> as_matrix(
    std::initializer_list<std::initializer_list<double>> data) {
  Eigen::Matrix<double, rows, cols> ret;
  int i = 0;
  for (auto il : data) {
    int j = 0;
    for (auto val : il) {
      ret(i, j) = val;
      j++;
    }
    i++;
  }
  return ret;
}
}

#ifndef MUAN_CONTROL_STATE_OBSERVER_H_
#define MUAN_CONTROL_STATE_OBSERVER_H_

#include "Eigen/Core"
#include "unitscpp/unitscpp.h"
#include "state_space_plant.h"

namespace muan {

template <int Inputs, int States, int Outputs>
class StateObserver {
 public:
  StateObserver(Eigen::Matrix<double, States, States> a, Eigen::Matrix<double, States, Outputs> b,
                  Eigen::Matrix<double, Inputs, States> c, Eigen::Matrix<double, States, Inputs> l)
      : plant_(a, b, c), l_(l) {}

  void Update(const Eigen::Vector<double, Outputs, 1>& u, const Eigen::Matrix<double, Inputs, 1>& y) {
    plant_.Update();
    plant_.SetX(plant.GetX() + l_*(y - plant.GetY()));
  }

  Eigen::Matrix<double, States, 1> GetX() { return plant_.GetX(); }

 private:
  StateSpacePlant<Inputs, States, Outputs> plant_;
  Eigen::Matrix<double, States, Inputs> l_;
};

}

#endif

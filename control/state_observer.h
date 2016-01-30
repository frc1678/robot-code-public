#ifndef MUAN_CONTROL_STATE_OBSERVER_H_
#define MUAN_CONTROL_STATE_OBSERVER_H_

#include "Eigen/Core"
#include "unitscpp/unitscpp.h"
#include "state_space_plant.h"

namespace muan {

template <int Inputs, int States, int Outputs, bool discrete = false>
class StateObserver {
 public:
  StateObserver(Time dt) : plant_(dt) {}
  StateObserver(StateSpacePlant<Inputs, States, Outputs, true> plant)
      : plant_(plant) {}

  void Update(const Eigen::Matrix<double, Outputs, 1>& u,
              const Eigen::Matrix<double, Inputs, 1>& y) {
    plant_.Update(u);
    plant_.SetX(plant_.GetX() +
                plant_.GetTimestep().to(s) * l_ * (y - plant_.GetY()));
  }

  void Update(const Eigen::Matrix<double, Outputs, 1>& u,
              const Eigen::Matrix<double, Inputs, 1>& y, Time dt) {
    plant_.Update(u, dt);
    plant_.SetX(plant_.GetX() +
                plant_.GetTimestep().to(s) * l_ * (y - plant_.GetY()));
  }

  Eigen::Matrix<double, States, 1> GetX() { return plant_.GetX(); }
  void SetX(Eigen::Matrix<double, States, 1> x) { plant_.SetX(x); }

  void SetGains(Eigen::Matrix<double, States, Inputs> l) { l_ = l; }
  void SetPlant(StateSpacePlant<Inputs, States, Outputs, discrete> plant) {
    plant_ = plant;
  }
  StateSpacePlant<Inputs, States, Outputs, discrete> GetPlant() {
    return plant_;
  }

 private:
  StateSpacePlant<Inputs, States, Outputs, discrete> plant_;
  Eigen::Matrix<double, States, Inputs> l_;
};
}

#endif

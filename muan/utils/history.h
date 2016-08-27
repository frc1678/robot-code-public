#ifndef MUAN_CONTROL_HISTORY_H_
#define MUAN_CONTROL_HISTORY_H_

#include "muan/units/units.h"
#include <stdexcept>

namespace muan {

template <int size>
class History {
  using namespace muan::units;

 public:
  History(Seconds time_step_) : current_pos_(0), time_step_(time_step_) {
    hist_arr_ = new double[size];
  }
  void Update(double val) {
    hist_arr_[current_pos_] = val;
    current_pos_ = (current_pos_ + 1) % size;
  }

  const double& GoBack(Seconds t) {
    if (t > time_step_ * size)
      throw std::out_of_range("Cannot go back to unrecorded history!");
    unsigned int element_pos =
        (current_pos_ - static_cast<int>(t.to(time_step_)) + size) % size;
    return hist_arr_[element_pos];
  }

  double* begin() { return &hist_arr_[0]; }
  double* end() { return &hist_arr_[size]; }

 private:
  int current_pos_;
  Seconds time_step_;
  double* hist_arr_;
};
}

#endif /* MUAN_CONTROL_HISTORY_H_ */

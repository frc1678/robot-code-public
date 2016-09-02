#ifndef MUAN_CONTROL_HISTORY_H_
#define MUAN_CONTROL_HISTORY_H_

#include "muan/units/units.h"
#include <stdexcept>

namespace muan {

template <class T, int size>
class History {
 public:
  History(muan::units::Time time_step_)
      : current_pos_(0), time_step_(time_step_) {
    hist_arr_ = new T[size];
  }
  void Update(T val) {
    hist_arr_[current_pos_] = val;
    current_pos_ = (current_pos_ + 1) % size;
  }

  const T& GoBack(muan::units::Time t) {
    if (t > time_step_ * size)
      throw std::out_of_range("Cannot go back to unrecorded history!");
    unsigned int element_pos =
        (current_pos_ - static_cast<int>(muan::units::convert(t, time_step_)) +
         size) %
        size;
    return hist_arr_[element_pos];
  }

  T* begin() { return &hist_arr_[0]; }
  T* end() { return &hist_arr_[size]; }

 private:
  int current_pos_;
  muan::units::Time time_step_;
  T* hist_arr_;
};
}

#endif /* MUAN_CONTROL_HISTORY_H_ */

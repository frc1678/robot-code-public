#ifndef MUAN_CONTROL_HISTORY_H_
#define MUAN_CONTROL_HISTORY_H_

#include "unitscpp/unitscpp.h"
#include <stdexcept>

template <class T, int size>
class History {
 public:
  History(Time time_step_) : current_pos_(0), time_step_(time_step_) {}

  void Update(T val) {
    hist_arr_[current_pos_] = val;
    current_pos_ = (current_pos_ + 1) % size;
  }

  const T& GoBack(Time t) {
    if (t > time_step_ * size)
      throw std::out_of_range("Cannot go back to unrecorded history!");
    unsigned int element_pos =
        (current_pos_ - static_cast<int>(t.to(time_step_)) + size) % size;
    return hist_arr_[element_pos];
  }

 private:
  int current_pos_;
  Time time_step_;
  T hist_arr_[size];
};

#endif /* MUAN_CONTROL_HISTORY_H_ */

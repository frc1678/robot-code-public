#ifndef MUAN_CALIBRATION_H_
#define MUAN_CALIBRATION_H_

namespace muan {

/*
 * A base class for calibrating an main sensor using an auxiliary sensor. The
 * Update(MainValueType main_value, AuxValueType aux_value) must be
 * implemented for each sensor pair.
 */
template <typename MainValueType, typename AuxValueType>
class Calibration {
  public:
    Calibration();
    ~Calibration();
    virtual void Update(MainValueType main_value, AuxValueType aux_value) = 0;
    bool Calibrated();
    MainValueType Offset();

  protected:
    bool calibrated_;
    MainValueType offset_;
};
}

#include "calibration.hpp"

#endif // MUAN_CALIBRATION_H_

namespace muan {

template <typename MainValueType, typename AuxValueType>
Calibration<MainValueType, AuxValueType>::Calibration() : calibrated_(false), offset_(0) {}

template <typename MainValueType, typename AuxValueType>
Calibration<MainValueType, AuxValueType>::~Calibration() {}

template <typename MainValueType, typename AuxValueType>
bool Calibration<MainValueType, AuxValueType>::Calibrated() { return calibrated_; }

template <typename MainValueType, typename AuxValueType>
MainValueType Calibration<MainValueType, AuxValueType>::Offset() { return offset_; }
}

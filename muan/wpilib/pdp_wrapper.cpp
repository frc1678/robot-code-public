#include "muan/wpilib/pdp_wrapper.h"
#include "third_party/aos/common/die.h"

namespace muan {

namespace wpilib {

PdpWrapper::PdpWrapper(int module) : module_{module} {
  int status;
  HAL_InitializePDP(module_, &status);

  if (status != 0) {
    std::cerr << "Error in PDP wrapper init: " << HAL_GetErrorMessage(status) << std::endl;
    num_failures_++;
  }
}

void PdpWrapper::SendValues() {
  PdpMessage message;

  int status;

  message->set_current0(HAL_GetPDPChannelCurrent(module_, 0, &status));
  message->set_current1(HAL_GetPDPChannelCurrent(module_, 1, &status));
  message->set_current2(HAL_GetPDPChannelCurrent(module_, 2, &status));
  message->set_current3(HAL_GetPDPChannelCurrent(module_, 3, &status));
  message->set_current4(HAL_GetPDPChannelCurrent(module_, 4, &status));
  message->set_current5(HAL_GetPDPChannelCurrent(module_, 5, &status));
  message->set_current6(HAL_GetPDPChannelCurrent(module_, 6, &status));
  message->set_current7(HAL_GetPDPChannelCurrent(module_, 7, &status));
  message->set_current8(HAL_GetPDPChannelCurrent(module_, 8, &status));
  message->set_current9(HAL_GetPDPChannelCurrent(module_, 9, &status));
  message->set_current10(HAL_GetPDPChannelCurrent(module_, 10, &status));
  message->set_current11(HAL_GetPDPChannelCurrent(module_, 11, &status));
  message->set_current12(HAL_GetPDPChannelCurrent(module_, 12, &status));
  message->set_current13(HAL_GetPDPChannelCurrent(module_, 13, &status));
  message->set_current14(HAL_GetPDPChannelCurrent(module_, 14, &status));
  message->set_current15(HAL_GetPDPChannelCurrent(module_, 15, &status));

  message->set_voltage_in(HAL_GetPDPVoltage(module_, &status));
  message->set_temperature(HAL_GetPDPTemperature(module_, &status));

  if (status != 0) {
    std::cerr << "Error in PDP wrapper: " << HAL_GetErrorMessage(status) << std::endl;
    num_failures_++;
  }

  message->set_num_failures(num_failures_);

  if (queue_ != nullptr) {
    queue_->WriteMessage(message);
  } else {
    aos::Die("PDP queue not set!");
  }
}

void PdpWrapper::SetQueue(Queue* pdp_queue) { queue_ = pdp_queue; }

}  // namespace wpilib

}  // namespace muan

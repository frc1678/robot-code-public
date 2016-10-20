#include "pdp_wrapper.h"

namespace muan {

namespace wpilib {

void PdpWrapper::SendValues() {
  PdpMessage message;
  for (size_t channel = 0; channel < 16; channel++) {
    message->add_current(pdp_.GetCurrent(channel));
  }

  message->set_voltage_in(pdp_.GetVoltage());
  message->set_temperature(pdp_.GetTemperature());
  message->set_total_current(pdp_.GetTotalCurrent());
  message->set_total_energy(pdp_.GetTotalEnergy());
  message->set_total_power(pdp_.GetTotalPower());

  if (queue_ != nullptr) {
    queue_->WriteMessage(message);
  }
}

void PdpWrapper::SetQueue(Queue* pdp_queue) { queue_ = pdp_queue; }

}  // wpilib

}  // muan

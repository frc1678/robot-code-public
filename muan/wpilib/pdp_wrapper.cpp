#include "muan/wpilib/pdp_wrapper.h"
#include "third_party/aos/common/die.h"

namespace muan {

namespace wpilib {

void PdpWrapper::SendValues() {
  PdpMessage message;

  if (pdp_.StatusIsFatal()) {
    pdp_.ClearError();
  }

  message->set_current0(pdp_.GetCurrent(0));
  message->set_current1(pdp_.GetCurrent(1));
  message->set_current2(pdp_.GetCurrent(2));
  message->set_current3(pdp_.GetCurrent(3));
  message->set_current4(pdp_.GetCurrent(4));
  message->set_current5(pdp_.GetCurrent(5));
  message->set_current6(pdp_.GetCurrent(6));
  message->set_current7(pdp_.GetCurrent(7));
  message->set_current8(pdp_.GetCurrent(8));
  message->set_current9(pdp_.GetCurrent(9));
  message->set_current10(pdp_.GetCurrent(10));
  message->set_current11(pdp_.GetCurrent(11));
  message->set_current12(pdp_.GetCurrent(12));
  message->set_current13(pdp_.GetCurrent(13));
  message->set_current14(pdp_.GetCurrent(14));
  message->set_current15(pdp_.GetCurrent(15));

  message->set_voltage_in(pdp_.GetVoltage());
  message->set_temperature(pdp_.GetTemperature());
  message->set_total_current(pdp_.GetTotalCurrent());
  message->set_total_energy(pdp_.GetTotalEnergy());
  message->set_total_power(pdp_.GetTotalPower());

  if (queue_ != nullptr) {
    queue_->WriteMessage(message);
  } else {
    aos::Die("PDP queue not set!");
  }
}

void PdpWrapper::SetQueue(Queue* pdp_queue) { queue_ = pdp_queue; }

}  // namespace wpilib

}  // namespace muan

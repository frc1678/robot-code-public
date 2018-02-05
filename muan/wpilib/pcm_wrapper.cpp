#include "muan/wpilib/pcm_wrapper.h"
#include "third_party/aos/common/die.h"

namespace muan {

namespace wpilib {

PcmWrapper::PcmWrapper(int32_t module) : module_(module) {}

PcmWrapper::PcmWrapper() : PcmWrapper(SensorBase::GetDefaultSolenoidModule()) {}

PcmWrapper::~PcmWrapper() {
  for (size_t i = 0; i < 8; i++) {
    if (handles_[i] != HAL_kInvalidHandle) {
      HAL_FreeSolenoidPort(handles_[i]);
    }
  }
}

bool PcmWrapper::CreateSolenoid(uint8_t port) {
  // Mark this channel as initialized
  if (!SensorBase::CheckSolenoidModule(module_)) {
    return false;
  }

  if (!SensorBase::CheckSolenoidChannel(port)) {
    return false;
  }

  int status = 0;
  HAL_SolenoidHandle handle =
      HAL_InitializeSolenoidPort(HAL_GetPortWithModule(module_, port), &status);
  if (status != 0) {
    return false;
  }

  handles_[port] = handle;

  return true;
}

void PcmWrapper::CreateDoubleSolenoid(uint8_t channel_forward,
                                      uint8_t channel_reverse) {
  CreateSolenoid(channel_forward);
  CreateSolenoid(channel_reverse);
}

void PcmWrapper::WriteDoubleSolenoid(uint8_t channel_forward,
                                     uint8_t channel_reverse,
                                     DoubleSolenoid::Value value) {
  SetChannel(channel_forward, value == DoubleSolenoid::Value::kForward);
  SetChannel(channel_reverse, value == DoubleSolenoid::Value::kReverse);
}

void PcmWrapper::WriteSolenoid(uint8_t channel, bool on) {
  SetChannel(channel, on);
}

void PcmWrapper::Flush() {
  // Write the cached values to CAN
  int status;
  HAL_SetAllSolenoids(module_, current_values_, &status);
  // TODO(Kyle) Do something with status if it isn't zero
}

void PcmWrapper::CheckPortInitialized(uint8_t channel) {
  if (handles_[channel] == HAL_kInvalidHandle) {
    aos::Die("Solenoid port %i not initialized!", channel);
  }
}

void PcmWrapper::SetChannel(uint8_t channel, bool on) {
  // Before we do anything, check that the port is initialized.
  CheckPortInitialized(channel);

  if (on) {
    // Set the bit
    current_values_ |= (1 << channel);
  } else {
    // Clear the bit
    current_values_ &= ~(1 << channel);
  }
}

}  // namespace wpilib

}  // namespace muan

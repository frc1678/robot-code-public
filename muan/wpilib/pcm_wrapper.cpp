#include "pcm_wrapper.h"
#include "third_party/aos/common/die.h"

namespace muan {

namespace wpilib {

void PcmWrapper::CreateSolenoid(uint8_t port) {
  // Mark this channel as initialized
  initialized_ |= (1 << port);
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
  SolenoidBase::Set(current_values_, 0xFF, m_moduleNumber);
}

void PcmWrapper::CheckPortInitialized(uint8_t channel) {
  if (!(initialized_ & (1 << channel))) {
    aos::Die("Solenoid port %i not initialized!", channel);
  }
}

void PcmWrapper::SetChannel(uint8_t channel, bool on) {
  // Before we do anything, check that the port is initialized. This isn't
  // really necessary, as WPILib automatically initializes all of the channels
  // on the PCM (as far as I can tell), but it is a nice check to make sure
  // people only write to solenoids that they actually mean to use.
  CheckPortInitialized(channel);

  if (on) {
    // Set the bit
    current_values_ |= (1 << channel);
  } else {
    // Clear the bit
    current_values_ &= ~(1 << channel);
  }
}

}  // wpilib

}  // muan

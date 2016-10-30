#ifndef MUAN_WPILIB_PCM_WRAPPER_H_
#define MUAN_WPILIB_PCM_WRAPPER_H_

#include "WPILib.h"
#include <atomic>
#include <cstdint>

namespace muan {

namespace wpilib {

class PcmWrapper : private SolenoidBase {
 public:
  PcmWrapper(uint8_t module = 0) : SolenoidBase(module) {}
  ~PcmWrapper() = default;

  // Create a solenoid on the specified channel.
  void CreateSolenoid(uint8_t port);
  void CreateDoubleSolenoid(uint8_t channel_forward, uint8_t channel_reverse);

  // Writes to the specified solenoid port. This is realtime and just sets
  // values that will be written in the CAN thread. These functions will die if
  // the corresponding channel is not initialized.
  void WriteSolenoid(uint8_t channel, bool on);
  void WriteDoubleSolenoid(uint8_t channel_forward, uint8_t channel_reverse,
                           DoubleSolenoid::Value value);

 private:
  friend class CanWrapper;

  // Flushes the current values to the CAN bus. This function is not realtime
  // and should only be called from the CAN thread. It's private so it can only
  // be called by friend classes, like CanWrapper.
  void Flush();
  // Make sure the port is initialized, and die if it is not
  void CheckPortInitialized(uint8_t port);

  void SetChannel(uint8_t channel, bool on);

  // Bitmasks of the initialized channels as well as the current values of each
  // solenoid
  std::atomic<uint8_t> initialized_{0}, current_values_{0};
};

}  // wpilib

}  // muan

#endif  // MUAN_WPILIB_PCM_WRAPPER_H_

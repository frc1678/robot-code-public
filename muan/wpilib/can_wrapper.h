#ifndef MUAN_WPILIB_CAN_WRAPPER_H_
#define MUAN_WPILIB_CAN_WRAPPER_H_

#include "pcm_wrapper.h"
#include "pdp_wrapper.h"

namespace muan {

namespace wpilib {

/*
 * A class to keep CAN bus calls far, far away from anything that ever needs to
 * be realtime.
 * Example:
 *  CanWrapper can;
 *  can.pdp()->SetQueue(&pdp_queue);
 *  can.pcm()->CreateSolenoid(0);
 *  std::thread can_thread(can);
 *  ...
 *  can.GetPcm()->WriteSolenoid(0, true);
 *  auto pdp_values = pdp_queue_reader.ReadMessage();
 */
class CanWrapper {
 public:
  CanWrapper() = default;
  ~CanWrapper() = default;

  // Call this to run the loop forever (or until Stop() is called)
  void operator()();

  PdpWrapper* pdp();
  PcmWrapper* pcm();

  void Stop();

 private:
  PdpWrapper pdp_;
  PcmWrapper pcm_;

  std::atomic<bool> running_{false};
};

}  // wpilib

}  // muan

#endif  // MUAN_WPILIB_CAN_WRAPPER_H_

#ifndef MUAN_TELEOP_BUTTON_H_
#define MUAN_TELEOP_BUTTON_H_

#include <cstdint>

namespace muan {

namespace teleop {

class Joystick;

class Button {
 public:
  bool was_clicked();
  bool was_released();
  bool is_pressed();

 protected:
  friend class muan::teleop::Joystick;

  Button(Joystick* joystick, uint32_t button);
  void Update();
  void Update(bool value);

  Joystick* joystick_;
  uint32_t id_;

 private:
  bool current_{false}, last_{false};
};

}  // teleop

}  // muan

#endif  // MUAN_TELEOP_BUTTON_H_

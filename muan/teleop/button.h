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
  virtual void Update();
  void Update(bool value);

  Joystick* joystick_;
  uint32_t id_;

 private:
  bool current_{false}, last_{false};
};

enum class Pov {
  kNorth = 0,
  kNorthEast = 45,
  kEast = 90,
  kSouthEast = 135,
  kSouth = 180,
  kSouthWest = 225,
  kWest = 270,
  kNorthWest = 315,
};

class PovButton : public Button {
 protected:
  friend class muan::teleop::Joystick;

  PovButton(Joystick* joystick, uint32_t pov, Pov position);
  void Update() override;

  Pov pov_position_;
};

class AxisButton : public Button {
 public:
  AxisButton(Joystick* joystick, uint32_t button, double trigger_threshold);
  void Update();

 private:
  double trigger_threshold_;
};

}  // namespace teleop

}  // namespace muan

#endif  // MUAN_TELEOP_BUTTON_H_

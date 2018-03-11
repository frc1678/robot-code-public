#ifndef MUAN_TELEOP_BUTTON_H_
#define MUAN_TELEOP_BUTTON_H_

#include <cstdint>

namespace muan {

namespace teleop {

class Joystick;

enum class Pov {
  kNorth = 0,
  kNorthEast = 45,
  kEast = 90,
  kSouthEast = 135,
  kSouth = 180,
  kSouthWest = 225,
  kWest = 270,
  kNorthWest = 315,
  kNorthAgain = 360
};

class Button {
 public:
  bool was_clicked();
  bool was_released();
  bool is_pressed();

  virtual void Update();

 protected:
  friend class muan::teleop::Joystick;

  Button(Joystick* joystick, uint32_t button);
  void Update(bool value);

  Joystick* joystick_;
  uint32_t id_;

 private:
  bool current_{false}, last_{false};
};

class PovButton : public Button {
 protected:
  friend class muan::teleop::Joystick;

  PovButton(Joystick* joystick, uint32_t pov, Pov position);

 public:
  void Update() override;

  Pov pov_position_;
};

class PovRange : public Button {
 public:
  friend class muan::teleop::Joystick;

  PovRange(Joystick* joystick, uint32_t button, double minimum, double maximum);

  void Update();

  int minimum_;
  int maximum_;
};

class AxisButton : public Button {
 public:
  AxisButton(Joystick* joystick, uint32_t button, double trigger_threshold);
  void Update();

 private:
  // Negative values will cause the button to be triggered when the axis drops
  // below the threshold, positive
  // values when it is above.
  double trigger_threshold_;
};

class AxisRange : public Button {
 public:
  friend class muan::teleop::Joystick;

  AxisRange(Joystick* joystick, double minimum, double maximum, double xaxis,
            double yaxis, double threshold);

  void Update();

  int minimum_;
  int maximum_;
  int yaxis_;
  int threshold_;
};

}  // namespace teleop

}  // namespace muan

#endif  // MUAN_TELEOP_BUTTON_H_

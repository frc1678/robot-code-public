#ifndef C2017_SUBSYSTEMS_LIGHTS_LIGHTS_H_
#define C2017_SUBSYSTEMS_LIGHTS_LIGHTS_H_

#include <string>
#include <vector>
#include "muan/wpilib/queue_types.h"
#include "muan/units/units.h"
#include "c2017/subsystems/lights/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"
#include "muan/webdash/queue_types.h"
#include "muan/webdash/webdash.pb.h"

namespace c2017 {

namespace lights {

enum class LightColor {
  OFF = 0,
  RED = 1,
  BLUE = 2,
  GREEN = 3,
  YELLOW = 4,
  TEAL = 5,
  PINK = 6,
  WHITE = 7,
};

class Lights {
 public:
  void Update();
  std::vector<std::string> auto_list;

 private:
  LightColor light_color_;
  LightColor VisionAlignment();
  LightColor FlashLights(LightColor, LightColor, bool);
  bool is_red() const;
  bool is_green() const;
  bool is_blue() const;
  bool calibrated_ = false;
  bool auto_running_ = false;
};

}  // namespace lights

}  // namespace c2017
#endif  // C2017_SUBSYSTEMS_LIGHTS_LIGHTS_H_

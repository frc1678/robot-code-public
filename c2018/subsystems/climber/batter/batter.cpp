#include "c2018/subsystems/climber/batter/batter.h"

namespace c2018 {

namespace climber {

namespace batter {

bool Batter::Update(bool put_down_batter, bool outputs_enabled) {
  return put_down_batter && outputs_enabled;
}

}  // namespace batter

}  // namespace climber

}  // namespace c2018

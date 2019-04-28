#ifndef C2019_COMMANDS_CARGO_SHIP_H_
#define C2019_COMMANDS_CARGO_SHIP_H_

#include "muan/logging/logger.h"
#include "c2019/commands/command_base.h"

namespace c2019 {
namespace commands {

class CargoShip : public c2019::commands::CommandBase {
 public:
  void LeftFrontCargoShip();
  void LeftSideCargoShip();
  void RightFrontCargoShip();
  void RightSideCargoShip();
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_CARGO_SHIP_H_

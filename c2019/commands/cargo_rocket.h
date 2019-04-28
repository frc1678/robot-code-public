#ifndef C2019_COMMANDS_CARGO_ROCKET_H_
#define C2019_COMMANDS_CARGO_ROCKET_H_

#include "muan/logging/logger.h"
#include "c2019/commands/command_base.h"

namespace c2019 {
namespace commands {

class CargoRocket : public c2019::commands::CommandBase {
 public:
  void LeftCargoRocket();
  void RightCargoRocket();
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_CARGO_ROCKET_H_

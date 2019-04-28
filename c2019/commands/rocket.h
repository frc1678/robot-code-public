#ifndef C2019_COMMANDS_ROCKET_H_
#define C2019_COMMANDS_ROCKET_H_

#include "muan/logging/logger.h"
#include "c2019/commands/command_base.h"

namespace c2019 {
namespace commands {

class Rocket : public c2019::commands::CommandBase {
 public:
  void LeftBackRocket();
  void LeftDoubleRocket();
  void RightBackRocket();
  void RightDoubleRocket();
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_ROCKET_H_

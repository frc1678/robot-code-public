#ifndef C2019_COMMANDS_DRIVE_STRAIGHT_H_
#define C2019_COMMANDS_DRIVE_STRAIGHT_H_

#include "muan/logging/logger.h"
#include "c2019/commands/command_base.h"

namespace c2019 {
namespace commands {

class DriveStraight : public c2019::commands::CommandBase {
 public:
  bool IsAutonomous() override;
  void operator()();
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_DRIVE_STRAIGHT_H_

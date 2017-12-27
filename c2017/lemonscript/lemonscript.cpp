#include "c2017/lemonscript/lemonscript.h"

#include <string>
#include "muan/utils/threading_utils.h"

namespace c2017 {
namespace lemonscript {

DEFINE_string(auto_mode, "none.auto", "What auto mode to default to if none is specified on the dashboard.");

Lemonscript::Lemonscript() {
  state_ = new ::lemonscript::LemonScriptState();
  decls_ =
      ::lemonscript::AvailableCppCommandDeclaration::parseCppCommands(AutoGenerator::GetAutoGenerators());
  decls_ =
      ::lemonscript::AvailableCppCommandDeclaration::parseCppCommands(AutoGenerator::GetAutoGenerators());
  state_->declareAvailableCppCommands(decls_);

  try {
    compiler_ = new ::lemonscript::LemonScriptCompiler("c2017/lemonscript/auto/" + FLAGS_auto_mode, state_);
  } catch (std::string e) {
    std::cerr << e << std::endl;
  }
}

Lemonscript::~Lemonscript() {
  delete compiler_;
  delete state_;
}

void Lemonscript::Start() { running_ = true; }
void Lemonscript::Stop() { running_ = false; }
void Lemonscript::Kill() { started_ = false; }

void Lemonscript::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(5));

  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("Lemonscript");

  running_ = false;
  started_ = true;
  while (started_) {
    if (running_) {
      running_ = !compiler_->PeriodicUpdate();
    } else {
      UpdateAutoRoutine();
    }
    phased_loop.SleepUntilNext();
  }
}

void Lemonscript::UpdateAutoRoutine() {
  auto message = auto_selection_reader_.ReadMessage();
  auto_list = c2017::QueueManager::GetInstance()->auto_list_;
  std::string filename = "none.auto";
  if (message) {
    if (message.value()->auto_mode() == auto_list[0]) {
      filename = "none.auto";
    } else if (message.value()->auto_mode() == auto_list[1]) {
      filename = "blue_hella_kpa.auto";
    } else if (message.value()->auto_mode() == auto_list[2]) {
      filename = "blue_hella_kpa_new.auto";
    } else if (message.value()->auto_mode() == auto_list[3]) {
      filename = "blue_hella_kpa_plus_gear.auto";
    } else if (message.value()->auto_mode() == auto_list[4]) {
      filename = "blue_center_peg_plus_kpa.auto";
    } else if (message.value()->auto_mode() == auto_list[5]) {
      filename = "blue_center_peg_plus_kpa_and_drive.auto";
    } else if (message.value()->auto_mode() == auto_list[6]) {
      filename = "blue_far_peg_plus_kpa_and_drive.auto";
    } else if (message.value()->auto_mode() == auto_list[7]) {
      filename = "red_hella_kpa.auto";
    } else if (message.value()->auto_mode() == auto_list[8]) {
      filename = "red_hella_kpa_new.auto";
    } else if (message.value()->auto_mode() == auto_list[9]) {
      filename = "red_hella_kpa_plus_gear.auto";
    } else if (message.value()->auto_mode() == auto_list[10]) {
      filename = "red_center_peg_plus_kpa.auto";
    } else if (message.value()->auto_mode() == auto_list[11]) {
      filename = "red_center_peg_plus_kpa_and_drive.auto";
    } else if (message.value()->auto_mode() == auto_list[12]) {
      filename = "red_far_peg_plus_kpa_and_drive.auto";
    } else if (message.value()->auto_mode() == auto_list[13]) {
      filename = "two_gear.auto";
    } else {
      filename = "none.auto";
    }
    std::cout << filename << std::endl;
    delete compiler_;
    try {
      compiler_ = new ::lemonscript::LemonScriptCompiler("c2017/lemonscript/auto/" + filename, state_);
    } catch (std::string s) {
      std::cerr << s << std::endl;
    }
  }
}

}  // namespace lemonscript
}  // namespace c2017

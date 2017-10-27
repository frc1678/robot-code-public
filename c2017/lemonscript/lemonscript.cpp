#include "c2017/lemonscript/lemonscript.h"

#include <string>

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
  aos::SetCurrentThreadName("Lemonscript");

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
  std::string filename = "none.auto";
  if (message) {
    if (message.value()->auto_mode() == "NONE") {
      filename = "none.auto";
    }
    else if (message.value()->auto_mode() == "TWO_GEAR") {
      filename = "two_gear.auto";
    }
    else if (message.value()->auto_mode() == "BLUE_HELLA_KPA") {
      filename = "blue_hella_kpa.auto";
    }
    else if (message.value()->auto_mode() == "BLUE_HELLA_KPA_NEW") {
      filename = "blue_hella_kpa_new.auto";
    }
    else if (message.value()->auto_mode() == "BLUE_HELLA_KPA_PLUS_GEAR") {
      filename = "blue_hella_kpa_plus_gear.auto";
    }
    else if (message.value()->auto_mode() == "BLUE_CENTER_PLUS_KPA") {
      filename = "blue_center_peg_plus_kpa.auto";
    }
    else if (message.value()->auto_mode() == "BLUE_CENTER_PLUS_KPA_DRIVE") {
      filename = "blue_center_peg_plus_kpa_and_drive.auto";
    }
    else if (message.value()->auto_mode() == "BLUE_FAR_PEG_PLUS_KPA_DRIVE") {
      filename = "blue_far_peg_plus_kpa_and_drive.auto";
    }
    else if (message.value()->auto_mode() == "RED_HELLA_KPA") {
      filename = "red_hella_kpa.auto";
    }
    else if (message.value()->auto_mode() ==  "RED_HELLA_KPA_NEW") {
      filename = "red_hella_kpa_new.auto";
    }
    else if (message.value()->auto_mode() == "RED_HELLA_KPA_PLUS_GEAR") {
      filename = "red_hella_kpa_plus_gear.auto";
    }
    else if (message.value()->auto_mode() == "RED_CENTER_PLUS_KPA") {
      filename = "red_center_peg_plus_kpa.auto";
    }
    else if (message.value()->auto_mode() == "RED_CENTER_PLUS_KPA_DRIVE") {
      filename = "red_center_peg_plus_kpa_and_drive.auto";
    }
    else if (message.value()->auto_mode() == "RED_FAR_PEG_PLUS_KPA_DRIVE") {
      filename = "red_far_peg_plus_kpa_and_drive.auto";
    }
    else if (message.value()->auto_mode() == "TWO_GEAR") {
      filename = "two_gear.auto";
    }
    else {
      filename = "none.auto";
    }
  std::cout << filename << std::endl;
  delete compiler_;
  compiler_ = new ::lemonscript::LemonScriptCompiler("c2017/lemonscript/auto/" + filename, state_);
  }
}

}  // namespace lemonscript
}  // namespace c2017

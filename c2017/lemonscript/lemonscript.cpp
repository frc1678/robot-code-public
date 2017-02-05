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
  auto message = webdash_reader_.ReadMessage();
  std::string filename = "none.auto";
  if (message) {
    switch (message.value()->auto_mode()) {
      case c2017::webdash::WebDash::NONE:
        filename = "none.auto";
        break;
      case c2017::webdash::WebDash::ONE_GEAR:
        filename = "one_gear.auto";
        break;
      case c2017::webdash::WebDash::TWO_GEAR:
        filename = "two_gear.auto";
        break;
      case c2017::webdash::WebDash::HELLA_KPA:
        filename = "hella_kpa.auto";
        break;
      case c2017::webdash::WebDash::HELLA_KPA_PLUS_GEAR:
        filename = "hella_kpa_plus_gear.auto";
        break;
      default:
        filename = "none.auto";
        break;
    }
    std::cout << filename << std::endl;
    delete compiler_;
    compiler_ = new ::lemonscript::LemonScriptCompiler(
        "c2017/lemonscript/auto/" + filename, state_);
  }
}

}  // namespace lemonscript
}  // namespace c2017

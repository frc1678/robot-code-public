#include "lemonscript.h"

namespace c2017 {

namespace lemonscript {

Lemonscript::Lemonscript() {
  state_ = new ::lemonscript::LemonScriptState();
  decls_ =
      ::lemonscript::AvailableCppCommandDeclaration::parseCppCommands(AutoGenerator::GetAutoGenerators());
  state_->declareAvailableCppCommands(decls_);
  try {
    compiler_ = new ::lemonscript::LemonScriptCompiler("test.auto", state_);
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
    }
    phased_loop.SleepUntilNext();
  }
}

}  // lemonscript
}  // c2017

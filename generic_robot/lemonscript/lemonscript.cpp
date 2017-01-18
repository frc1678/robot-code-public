#include "lemonscript.h"

namespace genericrobot {

namespace lemonscript {

Lemonscript::Lemonscript() {
  state_ = new ::lemonscript::LemonScriptState();
  decls_ = ::lemonscript::AvailableCppCommandDeclaration::parseCppCommands(AutoGenerator::GetAutoGenerators());
  state_->declareAvailableCppCommands(decls_);
  compiler_ = new ::lemonscript::LemonScriptCompiler("test.auto", state_);
}

Lemonscript::~Lemonscript() {
  delete compiler_;
  delete state_;
}

void Lemonscript::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(5));

  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("Lemonscript");

  running_ = true;
  while (running_) {
    running_ = !compiler_->PeriodicUpdate();
    phased_loop.SleepUntilNext();
  }

}

} // lemonscript

} // genericrobot

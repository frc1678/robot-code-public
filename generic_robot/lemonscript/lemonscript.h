#ifndef GENERIC_ROBOT_LEMONSCRIPT_LEMONSCRIPT_H_
#define GENERIC_ROBOT_LEMONSCRIPT_LEMONSCRIPT_H_

#include <string.h>
#include <atomic>
#include <vector>
#include "third_party/lemonscript_transpiler/example_func.h"
#include "third_party/lemonscript/lemonscript/lemonscript.h"
#include "third_party/lemonscript/lemonscript/AvailableCppCommandDeclaration.h"
#include "third_party/lemonscript/lemonscript/LemonScriptCompiler.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace genericrobot {

namespace lemonscript {

class Lemonscript {
 public:
  Lemonscript();
  ~Lemonscript();

  void operator()();

 private:
  ::lemonscript::LemonScriptState *state_;
  ::lemonscript::LemonScriptCompiler *compiler_;
  std::vector<const ::lemonscript::AvailableCppCommandDeclaration *> decls_;
  std::atomic<bool> running_;
};
}  // namespace lemonscript
}  // namespace genericrobot

#endif  // GENERIC_ROBOT_LEMONSCRIPT_LEMONSCRIPT_H_

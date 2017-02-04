#ifndef C2017_LEMONSCRIPT_LEMONSCRIPT_H_
#define C2017_LEMONSCRIPT_LEMONSCRIPT_H_

#include <string.h>
#include <atomic>
#include <vector>
#include "c2017/lemonscript/ls_gen.h"
#include "third_party/lemonscript/lemonscript/lemonscript.h"
#include "third_party/lemonscript/lemonscript/AvailableCppCommandDeclaration.h"
#include "third_party/lemonscript/lemonscript/LemonScriptCompiler.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2017 {
namespace lemonscript {

class Lemonscript {
 public:
  Lemonscript();
  ~Lemonscript();

  void operator()();

  void Start();  // Start running lemonscript
  void Stop();   // Pause running lemonscript
  void Kill();   // Stop the thread
 private:
  void UpdateAutoRoutine();
  c2017::webdash::WebDashQueue::QueueReader webdash_reader_ =
    QueueManager::GetInstance().webdash_queue().MakeReader();
  ::lemonscript::LemonScriptState *state_;
  ::lemonscript::LemonScriptCompiler *compiler_;
  std::vector<const ::lemonscript::AvailableCppCommandDeclaration *> decls_;
  std::atomic<bool> running_;
  std::atomic<bool> started_;
};

}  // namespace lemonscript
}  // namespace c2017

#endif  // C2017_LEMONSCRIPT_LEMONSCRIPT_H_

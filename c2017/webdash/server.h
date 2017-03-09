#ifndef C2017_WEBDASH_SERVER_H_
#define C2017_WEBDASH_SERVER_H_

#include <stdlib.h>
#include <signal.h>
#include <memory>
#include "third_party/seasocks/src/main/c/seasocks/Server.h"
#include "third_party/seasocks/src/main/c/seasocks/PrintfLogger.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace webdash {

class WebDashRunner {
 public:
  WebDashRunner() = default;
  ~WebDashRunner() = default;

  void operator()();
};

}  // namespace webdash
}  // namespace c2017

#endif  // C2017_WEBDASH_SERVER_H_

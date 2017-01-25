#ifndef C2017_WEBDASH_SERVER_H_
#define C2017_WEBDASH_SERVER_H_

#include <stdlib.h>
#include <signal.h>
#include "third_party/mongoose_cpp/upstream/Server.h"
#include "third_party/mongoose_cpp/upstream/WebController.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace webdash {

using Mongoose::RequestHandler;
using Mongoose::StreamResponse;

class WebDashController : public Mongoose::WebController {
 public:
  void save(Mongoose::Request &request, StreamResponse &response);
  void setup();
};

class WebDashRunner {
 public:
  WebDashRunner() = default;
  ~WebDashRunner() = default;

  void operator()();
 private:
  c2017::webdash::WebDashController controller_;
  Mongoose::Server server_{1678};
};

}  // namespace webdash
}  // namespace c2017

#endif  // C2017_WEBDASH_SERVER_H_

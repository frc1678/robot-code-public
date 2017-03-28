#ifndef C2017_WEBDASH_SERVER_H_
#define C2017_WEBDASH_SERVER_H_

#include <stdlib.h>
#include <signal.h>
#include <memory>
#include <vector>
#include <string>
#include <set>
#include "third_party/seasocks/src/main/c/seasocks/Server.h"
#include "third_party/seasocks/src/main/c/seasocks/PrintfLogger.h"
#include "third_party/optional/optional.hpp"
#include "third_party/aos/common/die.h"
#include "c2017/webdash/queue_types.h"

namespace c2017 {
namespace webdash {

class WebDashQueueWrapper {
 public:
  static WebDashQueueWrapper& GetInstance();
  c2017::webdash::AutoSelectionQueue& auto_selection_queue();
 private:
  c2017::webdash::AutoSelectionQueue auto_selection_queue_;
};

class WebDashRunner {
 public:
  WebDashRunner() = default;
  ~WebDashRunner() = default;

  template <class T>
  void AddQueue(const std::string& name, T* queue);

  class GenericReader {
   public:
    virtual std::experimental::optional<std::string> GetMessageAsJSON() = 0;
  };

  template <class R>
  class Reader : public GenericReader {
   public:
    explicit Reader(R reader);
    std::experimental::optional<std::string> GetMessageAsJSON() override;

   private:
    R reader_;
  };

  struct QueueLog {
    std::unique_ptr<GenericReader> reader;
    std::string name;  // Human friendly name for this log
  };

  void operator()();

 private:
  struct DataRequestHandler : seasocks::WebSocket::Handler {
   public:
    std::set<seasocks::WebSocket *> cons_;

    void onConnect(seasocks::WebSocket *con) override;
    void onDisconnect(seasocks::WebSocket *con) override;
    void onData(seasocks::WebSocket *con, const char * /*data*/) override;

    explicit DataRequestHandler(
        const std::vector<std::unique_ptr<webdash::WebDashRunner::QueueLog>> &queue_logs)
      : queue_logs_(&queue_logs) {}
   private:
    const std::vector<std::unique_ptr<webdash::WebDashRunner::QueueLog>> *queue_logs_;
  };

  std::vector<std::unique_ptr<QueueLog>> queue_logs_;
};

}  // namespace webdash
}  // namespace c2017

#include "server.hpp"

#endif  // C2017_WEBDASH_SERVER_H_

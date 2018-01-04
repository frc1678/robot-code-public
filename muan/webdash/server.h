#ifndef MUAN_WEBDASH_SERVER_H_
#define MUAN_WEBDASH_SERVER_H_

#include <signal.h>
#include <stdlib.h>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include "muan/webdash/queue_types.h"
#include "third_party/aos/common/die.h"
#include "third_party/optional/optional.hpp"
#include "third_party/seasocks/src/main/c/seasocks/PrintfLogger.h"
#include "third_party/seasocks/src/main/c/seasocks/Server.h"

namespace muan {
namespace webdash {

class WebDashQueueWrapper {
 public:
  static WebDashQueueWrapper &GetInstance();
  muan::webdash::AutoSelectionQueue &auto_selection_queue();

 private:
  muan::webdash::AutoSelectionQueue auto_selection_queue_;
};

class WebDashRunner {
 public:
  WebDashRunner() = default;
  ~WebDashRunner() = default;

  template <class T>
  void AddQueue(const std::string &name, T *queue);

  void AddAutos(const std::vector<std::string> *auto_list);

  const std::vector<std::string> *auto_list_;

  void DisplayObjectMaker(const std::string display_object);

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
    void onConnect(seasocks::WebSocket * /*con*/) override{};
    void onDisconnect(seasocks::WebSocket * /*con*/) override{};
    void onData(seasocks::WebSocket *con, const char * /*data*/) override;

    explicit DataRequestHandler(
        const std::vector<std::unique_ptr<webdash::WebDashRunner::QueueLog>> &queue_logs)
        : queue_logs_(&queue_logs) {}

   private:
    const std::vector<std::unique_ptr<webdash::WebDashRunner::QueueLog>> *queue_logs_;
  };

  struct AutoListRequestHandler : seasocks::WebSocket::Handler {
   public:
    std::set<seasocks::WebSocket *> cons_;

    void onConnect(seasocks::WebSocket * /*con*/) override{};
    void onDisconnect(seasocks::WebSocket * /*con*/) override{};
    void onData(seasocks::WebSocket *con, const char * /*data*/) override;

    explicit AutoListRequestHandler(const std::vector<std::string> *auto_list) : auto_list_(auto_list) {}

   private:
    const std::vector<std::string> *auto_list_;
  };

  struct DisplayRequestHandler : seasocks::WebSocket::Handler {
   public:
    void onConnect(seasocks::WebSocket * /*con*/) override{};
    void onDisconnect(seasocks::WebSocket * /*con*/) override{};
    void onData(seasocks::WebSocket *con, const char * /*data*/) override;

    explicit DisplayRequestHandler(const std::string *display_object) : display_object_(display_object) {}

   private:
    const std::string *display_object_;
  };

  std::string display_object_;
  std::vector<std::unique_ptr<QueueLog>> queue_logs_;
};

}  // namespace webdash
}  // namespace muan

#include "server.hpp"

#endif  // MUAN_WEBDASH_SERVER_H_

#ifndef MUAN_LOGGING_LOGGER_H_
#define MUAN_LOGGING_LOGGER_H_

#include "filewriter.h"
#include "muan/multithreading/updateable.h"
#include "muan/queues/message_queue.h"
#include "muan/units/units.h"
#include "muan/utils/proto_utils.h"
#include "third_party/optional/optional.hpp"

#include <iostream>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace muan {
namespace logging {

/*
 * A logging class that takes QueueReader objects and logs them to disk as CSV
 * files.
 *
 * Simple use:
 *
 * Logger logger;
 * MessageQueue<protobuf_class, 100> mq;
 * logger.AddQueue("some_queue", mq.MakeReader());
 *
 * You only want to have one logger object running at a time. You should add
 * QueueReaders to it as needed.
 */
class Logger : public muan::Updateable {
 public:
  Logger();
  Logger(std::shared_ptr<FileWriter> writer);
  virtual ~Logger() = default;

  // Adds a QueueReader<protobuf_class> to the list of queues to be logged,
  // under the name "name". The name will determine the file that it logs to,
  // as well as serving as a human-readable name in other places.
  template <class T>
  void AddQueue(std::string name, T& queue_reader);

  void Update(muan::units::Time dt) override;

 private:
  std::shared_ptr<FileWriter> writer_;

  class GenericReader {
   public:
    virtual std::experimental::optional<std::string> GetMessageAsCSV();
  };

  template <class T>
  class Reader : public GenericReader {
   public:
    Reader(T& reader);
    std::experimental::optional<std::string> GetMessageAsCSV() override;

   private:
    T& reader_;
  };

  struct QueueLog {
    std::unique_ptr<GenericReader> reader;
    std::string name;  // Human friendly name for this log
  };

  std::vector<std::unique_ptr<QueueLog>> all_logs_;

};  // class Logger

}  // namespace logging
}  // namespace muan

#endif

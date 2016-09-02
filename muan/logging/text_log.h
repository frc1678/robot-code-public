/*
 * text_log.h
 *
 *  Created on: Oct 2, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle
 */

#ifndef MUAN_LOGGING_TEXT_LOG_H_
#define MUAN_LOGGING_TEXT_LOG_H_

#include "log.h"
#include <string>

namespace muan {

/*
 * A class for logging plaintext messages.
 * Example:
 *  TextLog log("test");
 *  log.Write("Auto state change to state drive", "AUTO", CODE_STAMP);
 * Also supports static access of logs:
 *  TextLog::WriteToLog("test", "Auto state change to state drive", "AUTO",
 * CODE_STAMP);
 */
class TextLog : public Log {
  std::stringstream buffer_;

 public:
  explicit TextLog(std::string name);
  virtual void Write(std::string message, std::string category,
                     std::string code_stamp);
  void FlushToFile() override;
  std::string GetExtension() const override;
  static void WriteToLog(std::string log, std::string message,
                         std::string category, std::string code_stamp);
  ~TextLog() override;
};

}  // namespace muan

#endif /* MUAN_LOGGING_TEXT_LOG_H_ */

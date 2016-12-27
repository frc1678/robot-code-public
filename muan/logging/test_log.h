/*
 * test_log.h
 *
 *  Created on: Jan 21, 2016
 *  Copyright 2016 Citrus Circuits
 *      Author: Hiatt Leveillee
 */

#ifndef MUAN_LOGGING_TEST_LOG_H_
#define MUAN_LOGGING_TEST_LOG_H_

#include <string>
#include <utility>
#include <vector>
#include "log.h"

namespace muan {

/*
 * A logs the values from a test in a CSV (comma-separated values) format.
 * Example:
 *  TestLog log("filename", {"key1", "key2"});
 *  log["key1"] = "value1";
 *  log["key2"] = "value2";
 *  log.EndTest();
 */
class TestLog : public Log {
 public:
  explicit TestLog(std::string filename);
  virtual void Write(std::string key, std::string value);
  virtual void EndTest();
  virtual std::string &operator[](std::string key);

  void FlushToFile() override;
  std::string GetExtension() const override;

  static void WriteToLog(std::string log, std::string key, std::string value);
  ~TestLog() override = default;

 private:
  std::stringstream buffer_;
  std::vector<std::pair<std::string, std::string>> entries_;
};

}  // namespace muan

#endif /* MUAN_LOGGING_TEST_LOG_H_ */

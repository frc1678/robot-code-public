/*
 * log_manager.h
 *
 *  Created on: Oct 4, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#ifndef MUAN_LOGGING_LOG_MANAGER_H_
#define MUAN_LOGGING_LOG_MANAGER_H_

#include <atomic>
#include <map>
#include <string>
#include <thread>
#include "log.h"

namespace muan {

/*
 * A singleton object which manages all of an application's logs. Any instance
 * of a subclass of Log will automatically add itself to the LogManager.
 */
class LogManager {
  static LogManager *instance_;

  std::map<std::string, Log *> logs_;
  std::mutex logs_mutex_;
  std::thread run_thread_;
  std::atomic<bool> running_;
  void FlushLogs();

 public:
  LogManager();
  static LogManager *GetInstance();
  void AddLog(std::string key, Log *log);
  Log *GetLog(std::string key);
  void Stop();
  virtual ~LogManager();
};

}  // namespace muan

#endif /* MUAN_LOGGING_LOG_MANAGER_H_ */

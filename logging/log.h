/*
 * log.h
 *
 *  Created on: Oct 1, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#ifndef MUAN_LOGGING_LOG_H_
#define MUAN_LOGGING_LOG_H_

#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <mutex>
#include <map>

// #define LOG_VERBOSE

#ifdef LOG_VERBOSE
#define CODE_STAMP                                              \
  " (at line " + std::to_string(__LINE__) + " of " + __FILE__ + \
      " in function " + __PRETTY_FUNCTION__ + ")"
#else
#define CODE_STAMP ""
#endif

namespace muan {

/*
 * The base class for logs. Automatically flushes to a file every half second.
 * To use, implement FlushToFile() and GetExtension()
 */
class Log {
 public:
  Log(std::string name, std::string extension);
  virtual void FlushToFile() = 0;
  virtual std::string GetExtension() const = 0;
  virtual ~Log();

 protected:
  static std::string GetDateString();
  static std::string GetTimeString();

  static std::string folder_path_;
  static std::once_flag folder_created_;

  std::ofstream file_;
  std::mutex mutex_;
  std::string name_;
};

}

#endif  // MUAN_LOGGING_LOG_H_

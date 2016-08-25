/*
 * test_log.cpp
 *
 *  Created on: Jan 21, 2016
 *  Copyright 2016 Citrus Circuits
 *      Author: Hiatt Leveillee
 */

#include "test_log.h"
#include <string>
#include <vector>
#include "log_manager.h"

namespace muan {

TestLog::TestLog(std::string filename, std::vector<std::string> keys)
    : Log(filename, GetExtension()) {
  if (file_.is_open()) file_.close();
  folder_path_ = "./logs/tests/";
  file_.open(folder_path_ + name_ + ".csv");
}

/**
 * Write a value to a key in the test log for the current test.
 */
void TestLog::Write(std::string key, std::string value) {
  for (auto entry = entries_.begin(); entry != entries_.end(); entry++) {
    if (entry->first == key) {
      entry->second = value;
    }
  }
}

/**
 * Finish the current test and start a new one.
 */
void TestLog::EndTest() {
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_ << Log::GetTimeString() << ",";
  for (auto entry = entries_.begin(); entry != entries_.end(); entry++) {
    buffer_ << entry->second;
    if (entry + 1 != entries_.end()) {
      buffer_ << ",";
    }
    entry->second = "";
  }
  buffer_ << "\n";
}

/**
 * Save the current data to the log.
 */
void TestLog::FlushToFile() {
  std::lock_guard<std::mutex> lock(mutex_);
  file_ << buffer_.str();
  file_.flush();
  std::cout << buffer_.str() << std::endl;
  buffer_.str(std::string());
}

std::string TestLog::GetExtension() const { return "csv"; }

/**
 * Save the log to the disk.
 */
void TestLog::WriteToLog(std::string log, std::string key, std::string value) {
  reinterpret_cast<TestLog *>(LogManager::GetInstance()->GetLog(key))
      ->Write(key, value);
}

std::string &TestLog::operator[](std::string key) {
  for (auto it = entries_.begin(); it != entries_.end(); it++) {
    if (it->first == key) {
      return it->second;
    }
  }
  return entries_.begin()->second;
}

TestLog::~TestLog() {}
}

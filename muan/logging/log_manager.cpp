/*
 * log_manager.cpp
 *
 *  Created on: Oct 4, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "log_manager.h"
#include <string>

namespace muan {

LogManager *LogManager::instance_ = new LogManager();

LogManager::LogManager() {
  std::lock_guard<std::mutex> lock(logs_mutex_);
  running_ = true;
  run_thread_ = std::thread([this] {
    while (running_) {
      FlushLogs();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    FlushLogs();
  });
}

void LogManager::FlushLogs() {
  std::lock_guard<std::mutex> lock(logs_mutex_);
  for (auto logPair : GetInstance()->logs_) {
    logPair.second->FlushToFile();
  }
}

LogManager *LogManager::GetInstance() { return instance_; }

void LogManager::AddLog(std::string key, Log *log) {
  std::lock_guard<std::mutex> lock(logs_mutex_);
  if (logs_.find(key) == logs_.end()) {
    logs_[key] = log;
  }
}

Log *LogManager::GetLog(std::string key) {
  std::lock_guard<std::mutex> lock(logs_mutex_);
  if (logs_.find(key) != logs_.end()) {
    return logs_[key];
  }
  return nullptr;
}

void LogManager::Stop() {
  running_ = false;
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
}

LogManager::~LogManager() {
  Stop();

  for (auto pair : logs_) {
    delete pair.second;
  }
}

}  // namespace muan

/*
 * csv_log.cpp
 *
 *  Created on: Oct 2, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "csv_log.h"
#include <string>
#include <vector>
#include "log_manager.h"

namespace muan {

CSVLog::CSVLog(std::string filename, std::vector<std::string> keys)
    : Log(filename, GetExtension()) {
  buffer_ << "timestamp,";
  for (auto key = keys.begin(); key != keys.end(); key++) {
    buffer_ << *key;
    if (key + 1 != keys.end()) {
      buffer_ << ",";
    }
    this->entries_.push_back(std::make_pair(*key, ""));
  }
  buffer_ << "\n";
}

/**
 * Write a value to a key in the csv log for the current line.
 */
void CSVLog::Write(std::string key, std::string value) {
  for (auto entry = entries_.begin(); entry != entries_.end(); entry++) {
    if (entry->first == key) {
      entry->second = value;
    }
  }
}

/**
 * Begin writing values on a new line.
 */
void CSVLog::EndLine() {
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
void CSVLog::FlushToFile() {
  std::lock_guard<std::mutex> lock(mutex_);
  file_ << buffer_.str();
  file_.flush();
  buffer_.str(std::string());
}

std::string CSVLog::GetExtension() const { return "csv"; }

/**
 * Save the log to the disk.
 */
void CSVLog::WriteToLog(std::string log, std::string key, std::string value) {
  reinterpret_cast<CSVLog *>(LogManager::GetInstance()->GetLog(key))
      ->Write(key, value);
}

std::string &CSVLog::operator[](std::string key) {
  for (auto it = entries_.begin(); it != entries_.end(); it++) {
    if (it->first == key) {
      return it->second;
    }
  }
  return entries_.begin()->second;
}

CSVLog::~CSVLog() {}

const std::vector<std::pair<std::string, std::string>> CSVLog::GetEntries()
    const {
  return entries_;
}
}

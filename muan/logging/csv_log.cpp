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

CSVLog::CSVLog(std::string filename, std::vector<std::string> keys) : Log(filename, GetExtension()) {
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
  for (auto& entry : entries_) {
    if (entry.first == key) {
      entry.second = value;
    }
  }
}

/**
 * Begin writing values on a new line.
 */
void CSVLog::EndLine() {
  using namespace muan::units;
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_ << convert(timer.Get(), s) << ",";
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

std::string& CSVLog::operator[](std::string key) {
  for (auto& entry : entries_) {
    if (entry.first == key) {
      return entry.second;
    }
  }
  return entries_.begin()->second;
}

const std::vector<std::pair<std::string, std::string>> CSVLog::GetEntries() const { return entries_; }
}  // namespace muan

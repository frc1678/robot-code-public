/*
 * text_log.cpp
 *
 *  Created on: Oct 2, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "text_log.h"
#include "log_manager.h"
#include <string>

namespace muan {

TextLog::TextLog(std::string name) : Log(name, GetExtension()) {}

std::string TextLog::GetExtension() const { return "log"; }

/**
 * Write a message to the log with a tag.
 * Syntax: Write("My message.", "TAG", CODE_STAMP)
 */
void TextLog::Write(std::string message, std::string category, std::string code_stamp) {
  std::string full_message =
      "[" + category + "@" + Log::GetTimeString() + "] " + message + code_stamp + " in thread ";
  std::thread::id tid = std::this_thread::get_id();

  std::lock_guard<std::mutex> lock(mutex_);
  buffer_ << full_message << tid << "\n";
}

void TextLog::FlushToFile() {
  std::lock_guard<std::mutex> lock(mutex_);
  file_ << buffer_.str();
  file_.flush();
  buffer_.str(std::string());
}

}  // namespace muan

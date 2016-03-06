/*
 * log.cpp
 *
 *  Created on: Oct 1, 2015
 *  Copyright 2015 Citrus Circuits
 *      Author: Kyle Stachowicz
 */

#include "log.h"
#include <sys/time.h>
#include <sys/stat.h>
#include <ctime>
#include <string>
#include "log_manager.h"

namespace muan {

std::string Log::folder_path_ = "./logs/" + Log::GetDateString() + "/";
std::once_flag Log::folder_created_;

Log::Log(std::string name, std::string extension) {
  name_ = name;
  std::call_once(folder_created_, [=]() {
    mkdir(folder_path_.c_str(), 0777);
    chmod(folder_path_.c_str(), 0777);
  });

  std::string file_path = folder_path_ + name_ + "." + extension;

  file_.open(file_path);
  LogManager::GetInstance()->AddLog(name_ + "." + extension, this);
}

Log::~Log() { file_.close(); }

/**
 * Gets a srting with the date in the format YYYY-MM-DD_HH-MM-SS
 */
std::string Log::GetDateString() {
  time_t rawtime;
  tm* timeinfo;
  char buffer[80];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%Y-%m-%d_%H-%M-%S", timeinfo);
  return buffer;
}

/**
 * Gets a srting with the time in the format HH-MM-SS
 */
std::string Log::GetTimeString() {
  time_t rawtime;
  tm* timeinfo;
  char buffer[80];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%H-%M-%S", timeinfo);
  return buffer;
}

const std::string& Log::GetName() const { return name_; }
}

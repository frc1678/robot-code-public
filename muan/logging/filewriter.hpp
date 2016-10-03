#ifndef MUAN_LOGGING_FILEWRITER_HPP_
#define MUAN_LOGGING_FILEWRITER_HPP_

#include "filewriter.h"

namespace muan {
namespace logging {

FileWriter::FileWriter() {
  base_path_ = GetBasePath();
}

FileWriter::FileWriter(std::string base_path) {
  base_path_ = base_path;
}

void FileWriter::WriteLine(std::string filename, std::string line) {
  boost::filesystem::create_directories(filename);
  if (open_files_.find(filename) == open_files_.end()) {
    open_files_[filename].open(base_path_ + filename, std::ios::app);
  }
  open_files_[filename] << line << "\n";
  open_files_[filename].flush();
}

std::string FileWriter::GetBasePath() {
  std::vector<std::string> paths = {
      "/mnt/something/logs/",  //TODO(Wesley) find actual path being used on the roborio
      "/home/lvuser/logs/",
      "/home/wesley/logs/",
      "/logs",
      "/"};

  //TODO(Wesley) Check for space and perms
  for (auto const path : paths) {
    if (boost::filesystem::is_directory(path)) {
      auto final_path = path + std::to_string(std::time(0)) + "/";
      boost::filesystem::create_directories(final_path);
      return final_path;
    }
  }

  return "/";  //TODO(Wesley) error!
}

}  // namespace logging
}  // namespace muan

#endif /* MUAN_LOGGING_FILEWRITER_HPP_ */

#include "filewriter.h"

namespace muan {
namespace logging {

FileWriter::FileWriter() {
  base_path_ = GetBasePath();
}

FileWriter::FileWriter(const ::std::string &base_path) {
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
      "/mnt/sda1/",
      "/home/lvuser/"};

  //TODO(Wesley) Check for space
  for (auto const path : paths) {
    if (boost::filesystem::is_directory(path)) {
      try {
        auto final_path = path + "logs/" + std::to_string(std::time(0)) + "/";
        boost::filesystem::create_directories(final_path);
        return final_path;
      } catch (const boost::filesystem::filesystem_error& e) {
        std::cout << "Error opening path for logging:\n" << e.code().message() << "\n";
      }
    }
  }

  std::cout << "Could not find valid path for logging! Attempting to use /, but most likely no logs will be created.\n";
  return "/";
}

}  // namespace logging
}  // namespace muan

#include "muan/logging/filewriter.h"

#include <string>
#include <vector>

namespace muan {
namespace logging {

DEFINE_string(log_dir, "", "What directory to put logs in. Defaults to the current unix timestamp.");

FileWriter::FileWriter(const std::string &base_path) { base_path_ = base_path; }

FileWriter::FileWriter() {
  if (base_path_.empty()) {
    DetermineBasePath();
  }
}

void FileWriter::WriteLine(const std::string &filename, const std::string &line) {
  if (open_files_.find(filename) == open_files_.end()) {
    open_files_[filename].open((base_path_ / filename).string(), std::ios::app);
  }
  open_files_[filename] << line << "\n";
  open_files_[filename].flush();
}

void FileWriter::DetermineBasePath() {
  std::vector<boost::filesystem::path> paths = {"/media/sda1", "/home/lvuser"};

  // TODO(Wesley) Check for space
  for (auto const path : paths) {
    if (boost::filesystem::is_directory(path)) {
      try {
        std::string log_dir = FLAGS_log_dir.empty() ? std::to_string(std::time(0)) : FLAGS_log_dir;
        int path_number = 0;
        boost::filesystem::path final_path;
        while (boost::filesystem::is_directory(final_path = path / "logs" / (log_dir + "." + std::to_string(path_number)))) {
          path_number++;
        }
        boost::filesystem::create_directories(final_path);
        base_path_ = final_path;
        return;
      } catch (const boost::filesystem::filesystem_error &e) {
        std::cerr << "Error opening path for logging:\n" << e.code().message() << "\n";
      }
    }
  }

  std::cerr << "Could not find valid path for logging!\n"
               "Attempting to use /, but most likely no logs will be created.\n"
               "If you are seeing this error while running tests, use `bazel test` instead of `bazel run`, "
               "or pass --copt='-DFRC1678_NO_QUEUE_LOGGING'.\n";
  base_path_ = "/";
}

void FileWriter::CreateReadableName(std::string name) {
  auto readable_path = base_path_.parent_path() / name;
  if (boost::filesystem::exists(readable_path)) {
    boost::filesystem::remove(readable_path);
  }
  if (boost::filesystem::portable_posix_name(name)) {
    boost::filesystem::create_symlink(base_path_, readable_path);
  }
}

boost::filesystem::path FileWriter::base_path_;

}  // namespace logging
}  // namespace muan

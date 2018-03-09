#ifndef MUAN_LOGGING_FILEWRITER_H_
#define MUAN_LOGGING_FILEWRITER_H_

#include <ctime>
#include <fstream>
#include <map>
#include <string>
#include "boost/filesystem.hpp"
#include "gflags/gflags.h"

/*
 * This class deals with writing to files, making sure that the needed folders
 * exist. It is essentially an abstraction over the filesystem.
 *
 * The way it is designed to be used is by creating one object and calling the
 * WriteLine function multiple times.
 */

namespace muan {
namespace logging {

DECLARE_string(log_dir);

class FileWriter {
 public:
  FileWriter();
  explicit FileWriter(const std::string &base_path);
  virtual ~FileWriter() = default;
  // Writes a line to the file that can be referred to by 'filename'.
  // filename should not start with a slash. Calls with the same filename *to
  // the same instance of FileWriter* will always go to the same file.
  // However, calls with the same filename to different instances of
  // FileWriter may go to different files.
  virtual void WriteLine(const std::string &filename, const std::string &line);
  virtual void WriteBytes(const std::string &filename,
                          const std::string &bytes);
  virtual std::ostream &GetTextFile(const std::string &filename);
  virtual std::ostream &GetBinaryFile(const std::string &filename);

  void FlushAllFiles();

  // Creates a human-readable name for the current log directory. For example,
  // if the current log directory is /media/sda1/logs/12345678, and the desired
  // name is CVR_Quals_20, a symlink /media/sda1/logs/CVR_Quals_20 would be
  // created pointing to the current log directory. If the name is not a valid
  // POSIX filename, nothing happens. Overrwites existing file/directory with
  // the
  // same name.
  static void CreateReadableName(std::string name);

 private:
  static void DetermineBasePath();
  static boost::filesystem::path base_path_;
  std::map<std::string, std::ofstream> open_files_;
};

}  // namespace logging
}  // namespace muan

#endif  // MUAN_LOGGING_FILEWRITER_H_

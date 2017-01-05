#ifndef MUAN_LOGGING_FILEWRITER_H_
#define MUAN_LOGGING_FILEWRITER_H_

#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include "boost/filesystem.hpp"

/*
 * This class deals with writing to files, making sure that the needed folders
 * exist. It is essentially an abstraction over the filesystem.
 *
 * The way it is designed to be used is by creating one object and calling the
 * WriteLine function multiple times.
 */

namespace muan {
namespace logging {

class FileWriter {
 public:
  FileWriter();
  FileWriter(const std::string &base_path);
  virtual ~FileWriter() = default;
  // Writes a line to the file that can be referred to by 'filename'.
  // filename should not start with a slash. Calls with the same filename *to
  // the same instance of FileWriter* will always go to the same file.
  // However, calls with the same filename to different instances of
  // FileWriter may go to different files.
  virtual void WriteLine(const std::string &filename, const std::string &line);

 private:
  std::string GetBasePath();
  std::string base_path_ = "";
  std::map<std::string, std::ofstream> open_files_;
};

}  // namespace logging
}  // namespace muan

#endif /* MUAN_LOGGING_FILESYSTEM_H_ */

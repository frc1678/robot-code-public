#ifndef MUAN_UTILS_STRING_UTILS_H_
#define MUAN_UTILS_STRING_UTILS_H_

#include <cstdint>
#include <string>
#include <vector>

namespace muan {
namespace utils {

// Convert CamelCase to snake_case
std::size_t CamelToSnake(const char* bytes_in, std::size_t num_bytes_in,
                         char* bytes_out, std::size_t num_bytes_out);

std::vector<std::string> split(const std::string &s, char delim);

}  // namespace utils
}  // namespace muan

#endif  // MUAN_UTILS_STRING_UTILS_H_

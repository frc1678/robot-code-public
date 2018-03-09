#include "muan/utils/string_utils.h"

namespace muan {

namespace utils {

// Convert CamelCase to snake_case, returning string length
std::size_t CamelToSnake(const char* bytes_in, std::size_t num_bytes_in,
                         char* bytes_out, std::size_t num_bytes_out) {
  std::size_t writer_idx = 0;
  for (std::size_t idx = 0; idx < num_bytes_in && bytes_in[idx] != '\0' &&
                            writer_idx < num_bytes_out;
       idx++) {
    if ('A' <= bytes_in[idx] && bytes_in[idx] <= 'Z') {
      // Don't add an underscore in if it's the first letter
      if (idx > 0) {
        bytes_out[writer_idx] = '_';
        writer_idx++;
      }

      // Lowercase the letter
      bytes_out[writer_idx] = bytes_in[idx] - 'A' + 'a';
    } else {
      // Leave as-is
      bytes_out[writer_idx] = bytes_in[idx];
    }
    writer_idx++;
  }
  bytes_out[writer_idx] = '\0';
  return writer_idx;
}

}  // namespace utils

}  // namespace muan

#ifndef MUAN_UTILS_HASH_H_
#define MUAN_UTILS_HASH_H_

#include <cstdint>

namespace muan {

namespace utils {

struct hash_cstr {
  std::size_t operator()(const char* to_hash) const;
};

}  // namespace utils

}  // namespace muan

#endif  // MUAN_UTILS_HASH_H_

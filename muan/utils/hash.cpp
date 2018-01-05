#include "muan/utils/hash.h"

namespace muan {

namespace utils {

std::size_t hash_cstr::operator()(const char* to_hash) const {
  constexpr std::size_t kFnvOffsetBasis = static_cast<std::size_t>(0xcbf29ce484222325);
  constexpr std::size_t kFnvPrime = static_cast<std::size_t>(0x100000001b3);

  std::size_t hash = kFnvOffsetBasis;
  while (*to_hash != '\0') {
    hash = hash * kFnvPrime;
    hash = hash ^ *to_hash;
    to_hash++;
  }

  return hash;
}

}  // namespace utils

}  // namespace muan

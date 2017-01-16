#ifndef AOS_COMMON_CHECK_H_
#define AOS_COMMON_CHECK_H_

#include "third_party/aos/common/die.h"
#include <cstdio>

namespace aos {

#define CHECK(condition)                        \
  if (__builtin_expect(!(condition), 0)) {      \
    ::aos::Die("CHECK(%s) failed", #condition); \
  }

#define CHECK_OP(name, op, val1, val2)               \
  if (!__builtin_expect(val1 op val2, 1)) {          \
    ::aos::Die("CHECK(%s) failed", #val1 #op #val2); \
  }

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, <, val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, >, val1, val2)

inline int CheckSyscall(const char *syscall_string, int value) {
  if (__builtin_expect(value == -1, false)) {
    ::aos::Die("%s failed", syscall_string);
  }
  return value;
}

inline void CheckSyscallReturn(const char *syscall_string, int value) {
  if (__builtin_expect(value != 0, false)) {
    ::aos::Die("%s failed with %i", syscall_string, value);
  }
}

// Check that syscall does not return -1. If it does, PLOG(FATAL)s. This is
// useful for quickly checking syscalls where it's not very useful to print out
// the values of any of the arguments. Returns the result otherwise.
//
// Example: const int fd = PCHECK(open("/tmp/whatever", O_WRONLY))
#define PCHECK(syscall) ::aos::CheckSyscall(STRINGIFY(syscall), syscall)

// PELOG(FATAL)s with the result of syscall if it returns anything other than 0.
// This is useful for quickly checking things like many of the pthreads
// functions where it's not very useful to print out the values of any of the
// arguments.
//
// Example: PRCHECK(munmap(address, length))
#define PRCHECK(syscall) ::aos::CheckSyscallReturn(STRINGIFY(syscall), syscall)

}  // namespace aos

#endif

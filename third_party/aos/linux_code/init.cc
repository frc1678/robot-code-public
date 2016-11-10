#include "third_party/aos/linux_code/init.h"

#include <errno.h>
#include <malloc.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include "third_party/aos/common/check.h"
#include "third_party/aos/common/die.h"

namespace FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead {
extern double FLAGS_tcmalloc_release_rate __attribute__((weak));
}
using FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead::
    FLAGS_tcmalloc_release_rate;

namespace aos {
namespace {

void SetSoftRLimit(int resource, rlim64_t soft, bool set_for_root) {
  bool am_root = getuid() == 0;
  if (set_for_root || !am_root) {
    struct rlimit64 rlim;
    if (getrlimit64(resource, &rlim) == -1) {
      PDie("%s-init: getrlimit64(%d) failed", program_invocation_short_name,
           resource);
    }
    rlim.rlim_cur = soft;
    rlim.rlim_max = ::std::max(rlim.rlim_max, soft);

    if (setrlimit64(resource, &rlim) == -1) {
      PDie("%s-init: setrlimit64(%d, {cur=%ju,max=%ju}) failed",
           program_invocation_short_name, resource, (uintmax_t)rlim.rlim_cur,
           (uintmax_t)rlim.rlim_max);
    }
  }
}

// Common stuff that needs to happen at the beginning of both the realtime and
// non-realtime initialization sequences. May be called twice.
void InitStart() { WriteCoreDumps(); }

const char *const kNoRealtimeEnvironmentVariable = "AOS_NO_REALTIME";

}  // namespace

void LockAllMemory() {
  // Allow locking as much as we want into RAM.
  SetSoftRLimit(RLIMIT_MEMLOCK, RLIM_INFINITY, false);

  InitStart();
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    PDie("%s-init: mlockall failed", program_invocation_short_name);
  }

  // Don't give freed memory back to the OS.
  CHECK_EQ(1, mallopt(M_TRIM_THRESHOLD, -1));
  // Don't use mmap for large malloc chunks.
  CHECK_EQ(1, mallopt(M_MMAP_MAX, 0));

  if (&FLAGS_tcmalloc_release_rate) {
    // Tell tcmalloc not to return memory.
    FLAGS_tcmalloc_release_rate = 0.0;
  }

  // Forces the memory pages for all the stack space that we're ever going to
  // use to be loaded into memory (so it can be locked there).
  uint8_t data[4096 * 8];
  // Not 0 because linux might optimize that to a 0-filled page.
  memset(data, 1, sizeof(data));

  static const size_t kHeapPreallocSize = 512 * 1024;
  char *const heap_data = static_cast<char *>(malloc(kHeapPreallocSize));
  memset(heap_data, 1, kHeapPreallocSize);
  free(heap_data);
}

void InitNRT() {
  InitStart();
  // LOG(INFO, "%s initialized non-realtime\n", program_invocation_short_name);
}

void InitCreate() {
  InitStart();
  // LOG(INFO, "%s created shm\n", program_invocation_short_name);
}

void Init(int relative_priority) {
  bool realtime = getenv(kNoRealtimeEnvironmentVariable) == nullptr;
  if (realtime) {
    LockAllMemory();

    // Only let rt processes run for 3 seconds straight.
    SetSoftRLimit(RLIMIT_RTTIME, 3000000, true);

    // Allow rt processes up to priority 40.
    SetSoftRLimit(RLIMIT_RTPRIO, 40, false);

    // Set our process to the appropriate priority.
    struct sched_param param;
    param.sched_priority = 30 + relative_priority;
    /*
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      // This was making the code not run
      //PDie("%s-init: setting SCHED_FIFO failed", program_invocation_short_name);
    }
    */
  } else {
    fprintf(stderr,
            "%s not doing realtime initialization because environment"
            " variable %s is set\n",
            program_invocation_short_name, kNoRealtimeEnvironmentVariable);
    printf("no realtime for %s. see stderr\n", program_invocation_short_name);
  }

  InitStart();
  // LOG(INFO, "%s initialized realtime\n", program_invocation_short_name);
}

void Cleanup() {}

void WriteCoreDumps() {
  // Do create core files of unlimited size.
  SetSoftRLimit(RLIMIT_CORE, RLIM_INFINITY, true);
}

void SetCurrentThreadRealtimePriority(int priority) {
  // Make sure we will only be allowed to run for 3 seconds straight.
  SetSoftRLimit(RLIMIT_RTTIME, 3000000, true);

  struct sched_param param;
  param.sched_priority = priority;
  /*
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    // This was making the code not run
    //::aos::Die("sched_setscheduler(0, SCHED_FIFO, %d) failed", priority);
  }
  */
}

void PinCurrentThreadToCPU(int number) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(number, &cpuset);
  PRCHECK(pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset));
}

void SetCurrentThreadName(const ::std::string &name) {
  if (name.size() > 16) {
    ::aos::Die("thread name '%s' too long\n", name.c_str());
  }
  // LOG(INFO, "this thread is changing to '%s'\n", name.c_str());
  PCHECK(prctl(PR_SET_NAME, name.c_str()));
}

}  // namespace aos

#include "timer.h"
#include "timing_utils.h"

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

void Timer::Reset() { start_ = now(); }

Time Timer::Get() { return now() - start_; }

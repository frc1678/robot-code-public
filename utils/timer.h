#include "../unitscpp/unitscpp.h"

class Timer {
 public:
  Timer();
  void Start();
  void Reset();
  Time Get();

 private:
  Time start_;
};

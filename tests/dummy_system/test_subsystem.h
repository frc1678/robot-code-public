#include "../../multithreading/updateable.h"

using namespace muan;

class TestSubsystem : public Updateable {
 public:
  TestSubsystem() : Updateable(100 * hz) {
    count_ = 0;
    to_add_ = 0;
  }
  void Actuate() { to_add_++; }
  void Update(Time dt) override {
    count_ += to_add_;
    to_add_ = 0;
  }
  int count_, to_add_;
};

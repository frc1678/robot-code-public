#include "c2017/subsystems/superstructure/magazine/magazine.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "gtest/gtest.h"

class MagazineTest : public ::testing::Test {
 public:
  void Update() {
    magazine.Update(input);
  }

  void SetConveyorCurrent(double conveyor)


 private:
  c2017::magazine::Magazine magazine;
  
};

TEST_F(MagazineTest,CanRotateWithGear) {
  c2017::magazine::MagazineInputProto input;
  Update(input);
  
}

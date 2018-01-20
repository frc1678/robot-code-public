#include "test/gtest.h"
#include "c2018/subsystems/score_subsystems/"
#include "c2018/queuemanager/queuemanager.h"

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {}
  void Update() { score_subsystem_.Update(); }

  void ReadMessages() {
    

  }
 private:
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;
};
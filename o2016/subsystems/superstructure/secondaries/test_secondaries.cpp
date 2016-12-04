#include "gtest/gtest.h"
#include "queue_types.h"
#include "secondaries.h"

using namespace o2016::secondaries;

TEST(Secondaries, Down) {
  Secondaries secondaries;
  SecondariesGoalProto goal;

  goal->set_position(Position::DOWN);
  goal->set_direction(Direction::IDLE);

  auto output = secondaries.Update(goal);

  ASSERT_TRUE(output->is_down());
}

TEST(Secondaries, Up) {
  Secondaries secondaries;
  SecondariesGoalProto goal;

  goal->set_position(Position::UP);
  goal->set_direction(Direction::IDLE);

  auto output = secondaries.Update(goal);

  ASSERT_FALSE(output->is_down());
}

TEST(Secondaries, Idle) {
  Secondaries secondaries;
  SecondariesGoalProto goal;

  goal->set_position(Position::DOWN);
  goal->set_direction(Direction::IDLE);

  auto output = secondaries.Update(goal);

  ASSERT_EQ(output->voltage(), 0);
}

TEST(Secondaries, Forward) {
  Secondaries secondaries;
  SecondariesGoalProto goal;

  goal->set_position(Position::DOWN);
  goal->set_direction(Direction::FORWARD);

  auto output = secondaries.Update(goal);

  ASSERT_EQ(output->voltage(), 12);
}

TEST(Secondaries, Reverse) {
  Secondaries secondaries;
  SecondariesGoalProto goal;

  goal->set_position(Position::DOWN);
  goal->set_direction(Direction::REVERSE);

  auto output = secondaries.Update(goal);

  ASSERT_EQ(output->voltage(), -12);
}

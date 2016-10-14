#include "gtest/gtest.h"
#include "scoop.h"
#include "o2016/subsystems/superstructure/catapult/scoop/scoop_constants.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_observer.h"
#include <iostream>

using namespace muan::control;
using namespace frc1678::scoop;

TEST(Scoop, converges) {
  for (double initial = -2.; initial <= 2.; initial += .5) {
    for (double goal = -2.; goal <= 2.; goal += .5) {
      auto plant = StateSpacePlant<1, 2, 1> (controller::A(), controller::B(), controller::C());
      plant.x(0) = initial;

      Scoop s;
      s.set_angle(initial);
      for (int i = 0; i < 1000; i++) {
        double power = s.Update(goal, plant.y(0));
        EXPECT_NEAR(power, 0., 12.);
        auto u = (Eigen::Matrix<double, 1, 1>() << power).finished();
	plant.Update(u);
        std::cout << plant.x(0) << " " << power << std::endl;
      }
      EXPECT_NEAR(s.get_angle(), goal, 0.01);
      EXPECT_NEAR(s.get_angular_velocity(), 0., 0.01);
    }
  }
  std::cout << controller::A() << std::endl;
}

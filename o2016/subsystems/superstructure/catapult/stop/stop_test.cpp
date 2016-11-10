#include "gtest/gtest.h"
#include "stop.h"
#include "o2016/subsystems/superstructure/catapult/stop/stop_constants.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_observer.h"

using namespace muan::control;
using namespace frc1678::stop;
using namespace o2016::catapult;

TEST(CatapultStop, converges) {
  for (double initial = -10.; initial <= 10.; initial += 10.) {
    for (double goal = -10.; goal <= 10.; goal += 10.) {
      auto plant = StateSpacePlant<1, 2, 1> (controller::A(), controller::B(), controller::C());
      plant.x(0) = initial;

      CatapultStop s;
      s.set_angle(initial);
      for (int i = 0; i < 1000; i++) {
        double power = s.Update(goal, plant.y(0));
        EXPECT_NEAR(power, 0., 12.);
        auto u = (Eigen::Matrix<double, 1, 1>() << power).finished();
	plant.Update(u);
      }
      EXPECT_NEAR(s.get_angle(), goal, 0.01);
      EXPECT_NEAR(s.get_angular_velocity(), 0., 0.01);
      EXPECT_TRUE(s.is_done());
    }
  }
  std::cout << controller::A() << std::endl;
}

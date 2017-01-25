#ifndef MUAN_CONTROL_GENERATED_CPP_TEST_
#define MUAN_CONTROL_GENERATED_CPP_TEST_

#include "gtest/gtest.h"

#include "muan/control/cpp_test_constants.h"

#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"

namespace muan {

namespace control {

// Make sure the python code doesn't write transposed matrices (this is a
// concern because of column- vs. row- major arrays in python vs. c-family)
TEST(GeneratedCppControlsTest, CorrectTransposition) {
  ASSERT_EQ(frc1678::cpp_test::controller::A_c()(0, 1), 1.0);
  ASSERT_EQ(frc1678::cpp_test::controller::A_c()(1, 0), -1.0);
}

}  // namespace controls

}  // namespace muan

#endif  // ifndef MUAN_CONTROL_GENERATED_CPP_TEST_

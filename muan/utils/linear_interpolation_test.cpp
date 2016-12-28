#include "gtest/gtest.h"
#include "muan/utils/linear_interpolation.h"

using muan::utils::LinearInterpolation;

TEST(LinearInterpolation, VerifiesListSize) {
  EXPECT_DEATH(LinearInterpolation<double> f({std::make_pair(1., 1.)}),
               "Interpolate requires 2 or more control points");
}

TEST(LinearInterpolation, CalculatesCorrectly) {
  LinearInterpolation<double> f({std::make_pair(3., 2.),
                                 std::make_pair(1., 0.)});
  f.AddControlPoint(std::make_pair(0., 0.));
  f.AddControlPoint(std::make_pair(7., 7.));
  f.AddControlPoint(std::make_pair(6., 8.));
  EXPECT_EQ(f.lower_boundary(), 0.);
  EXPECT_EQ(f.upper_boundary(), 7.);
  EXPECT_EQ(f(0), 0);
  EXPECT_EQ(f(1), 0);
  EXPECT_EQ(f(2), 1);
  EXPECT_EQ(f(3), 2);
  EXPECT_EQ(f(4), 4);
  EXPECT_EQ(f(5), 6);
  EXPECT_EQ(f(6), 8);
  EXPECT_EQ(f(7), 7);
  EXPECT_DEATH(f(100), "An interpolation is only defined between the lowest and highest x-values");
}

// Copyright 2024 Deepak Prakash Kumar
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>

#include "dubins_path_3d/planar_dubins.hpp"

namespace
{

using dubins_path_3d::planar::Config;

constexpr double kTwoPi = 2.0 * M_PI;

double angleDifference(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

}  // namespace

TEST(PlanarDubins, StraightLineIsRecovered)
{
  const Config start{0.0, 0.0, 0.0};
  const Config goal{10.0, 0.0, 0.0};

  const auto solution = dubins_path_3d::planar::optimalPath(start, goal, 2.0);
  ASSERT_TRUE(solution.valid());
  EXPECT_NEAR(solution.length, 10.0, 1e-9);
}

TEST(PlanarDubins, KnownLslGeometry)
{
  // Half a left turn of radius 1 followed by nothing: the vehicle ends up two
  // radii to the left, heading backwards.
  const Config start{0.0, 0.0, 0.0};
  const Config goal{0.0, 2.0, M_PI};

  const auto solution = dubins_path_3d::planar::optimalPath(start, goal, 1.0);
  ASSERT_TRUE(solution.valid());
  EXPECT_NEAR(solution.length, M_PI, 1e-9);
}

TEST(PlanarDubins, EveryFamilyEndsAtTheRequestedConfiguration)
{
  std::mt19937 generator(20240517U);
  std::uniform_real_distribution<double> position(-20.0, 20.0);
  std::uniform_real_distribution<double> heading(0.0, kTwoPi);

  const double radius = 3.0;
  int feasible_count = 0;

  for (int trial = 0; trial < 400; ++trial) {
    const Config start{position(generator), position(generator), heading(generator)};
    const Config goal{position(generator), position(generator), heading(generator)};

    for (const std::string & type : dubins_path_3d::planar::pathTypes()) {
      const auto solution = (type[1] == 's') ?
        dubins_path_3d::planar::cscPath(start, goal, radius, type) :
        dubins_path_3d::planar::cccPath(start, goal, radius, type);
      if (!solution.valid()) {
        continue;
      }
      ++feasible_count;

      const Config reached =
        dubins_path_3d::planar::finalConfig(start, radius, solution.params, type);
      EXPECT_NEAR(reached.x, goal.x, 1e-6) << "type " << type;
      EXPECT_NEAR(reached.y, goal.y, 1e-6) << "type " << type;
      EXPECT_LT(angleDifference(reached.heading, goal.heading), 1e-6) << "type " << type;
    }
  }

  EXPECT_GT(feasible_count, 400) << "expected several feasible families per trial";
}

TEST(PlanarDubins, OptimalIsNoLongerThanAnyFamily)
{
  std::mt19937 generator(987654321U);
  std::uniform_real_distribution<double> position(-15.0, 15.0);
  std::uniform_real_distribution<double> heading(0.0, kTwoPi);

  const double radius = 2.5;
  for (int trial = 0; trial < 200; ++trial) {
    const Config start{position(generator), position(generator), heading(generator)};
    const Config goal{position(generator), position(generator), heading(generator)};

    const auto best = dubins_path_3d::planar::optimalPath(start, goal, radius);
    ASSERT_TRUE(best.valid());

    for (const std::string & type : dubins_path_3d::planar::pathTypes()) {
      const auto solution = (type[1] == 's') ?
        dubins_path_3d::planar::cscPath(start, goal, radius, type) :
        dubins_path_3d::planar::cccPath(start, goal, radius, type);
      if (solution.valid()) {
        EXPECT_LE(best.length, solution.length + 1e-9) << "type " << type;
      }
    }
  }
}

TEST(PlanarDubins, SamplesFollowTheRequestedSpacing)
{
  const Config start{0.0, 0.0, 0.0};
  const Config goal{40.0, 15.0, 1.0};
  const double radius = 4.0;

  const auto solution = dubins_path_3d::planar::optimalPath(start, goal, radius);
  ASSERT_TRUE(solution.valid());

  const auto samples =
    dubins_path_3d::planar::pathSamples(start, radius, solution.params, solution.type, 1.0);
  ASSERT_GE(samples.size(), 2U);

  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double step = std::hypot(
      samples[i].x - samples[i - 1].x, samples[i].y - samples[i - 1].y);
    EXPECT_LT(step, 2.0) << "gap between samples " << i - 1 << " and " << i;
  }

  EXPECT_NEAR(samples.front().x, start.x, 1e-9);
  EXPECT_NEAR(samples.front().y, start.y, 1e-9);
}

TEST(PlanarDubins, LowerBoundedByStraightLineDistance)
{
  std::mt19937 generator(13579U);
  std::uniform_real_distribution<double> position(-30.0, 30.0);
  std::uniform_real_distribution<double> heading(0.0, kTwoPi);

  for (int trial = 0; trial < 200; ++trial) {
    const Config start{position(generator), position(generator), heading(generator)};
    const Config goal{position(generator), position(generator), heading(generator)};

    const auto best = dubins_path_3d::planar::optimalPath(start, goal, 2.0);
    ASSERT_TRUE(best.valid());
    EXPECT_GE(best.length, std::hypot(goal.x - start.x, goal.y - start.y) - 1e-9);
  }
}

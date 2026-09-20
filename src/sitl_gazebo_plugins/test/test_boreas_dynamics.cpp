#include <gtest/gtest.h>

#include <cmath>

#include "sitl_gazebo_plugins/boreas_dynamics.hpp"

namespace
{
ethrc::sitl::PropulsionConfig TestConfig()
{
  ethrc::sitl::PropulsionConfig config;
  config.thrustCurveN = {0.0, 0.0, 8.0};
  config.reactionMomentRatioM = 0.01;
  config.maximumRpm = 30000.0;
  config.timeConstantUpS = 0.025;
  config.timeConstantDownS = 0.050;
  return config;
}
}  // namespace

TEST(RotorDynamics, StartsStoppedAndClampsCommands)
{
  ethrc::sitl::RotorDynamics rotor(TestConfig());
  EXPECT_DOUBLE_EQ(rotor.NormalizedSpeed(), 0.0);
  rotor.Step(2.0, 1.0);
  EXPECT_NEAR(rotor.NormalizedSpeed(), 1.0, 1e-12);
  EXPECT_NEAR(rotor.ThrustN(), 8.0, 1e-10);
  EXPECT_NEAR(rotor.Rpm(), 30000.0, 1e-8);
}

TEST(RotorDynamics, UsesExactFirstOrderResponse)
{
  ethrc::sitl::RotorDynamics rotor(TestConfig());
  rotor.Step(1.0, 0.025);
  EXPECT_NEAR(rotor.NormalizedSpeed(), 1.0 - std::exp(-1.0), 1e-12);
}

TEST(RotorDynamics, AppliesReactionTorqueDirection)
{
  ethrc::sitl::RotorDynamics rotor(TestConfig());
  rotor.Step(1.0, 1.0);
  EXPECT_NEAR(rotor.ReactionTorqueNm(1.0), 0.08, 1e-10);
  EXPECT_NEAR(rotor.ReactionTorqueNm(-1.0), -0.08, 1e-10);
}

TEST(RotorDynamics, RejectsInvalidSteps)
{
  ethrc::sitl::RotorDynamics rotor(TestConfig());
  rotor.Step(1.0, 0.0);
  rotor.Step(NAN, 0.1);
  EXPECT_DOUBLE_EQ(rotor.NormalizedSpeed(), 0.0);
}

TEST(QuadraticDrag, OpposesMotionPerAxis)
{
  const auto drag = ethrc::sitl::QuadraticDrag(
      {2.0, -3.0, 0.0}, {0.5, 0.25, 1.0});
  EXPECT_DOUBLE_EQ(drag[0], -2.0);
  EXPECT_DOUBLE_EQ(drag[1], 2.25);
  EXPECT_DOUBLE_EQ(drag[2], 0.0);
}

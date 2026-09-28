#include <gtest/gtest.h>

#include <cmath>

#include "cartesian_manager/core/manager.hpp"
#include "cartesian_manager/core/shapers/behaviour/intent_scaling.hpp"

using manager_core::CartesianCommand;
using manager_core::IntentScaling;
using manager_core::IntentScalingConfig;
using manager_core::RobotContext;

namespace
{
  constexpr double kDt = 0.01;

  CartesianCommand push(double x, double y = 0.0, double z = 0.0)
  {
    CartesianCommand command;
    command.linear = Eigen::Vector3d(x, y, z);
    return command;
  }

  // Runs the shaper for `seconds` on the same command and returns the last output.
  CartesianCommand hold(IntentScaling &shaper, const CartesianCommand &input, double seconds)
  {
    CartesianCommand output;
    const RobotContext context;
    for (int tick = 0; tick < static_cast<int>(std::lround(seconds / kDt)); ++tick)
      output = shaper.update(input, context, kDt);
    return output;
  }
} // namespace

TEST(IntentScaling, StartsAPushAtTheMinimumScale)
{
  IntentScaling shaper{IntentScalingConfig{}};
  const auto output = shaper.update(push(1.0), RobotContext{}, kDt);
  EXPECT_NEAR(output.linear.x(), 0.4, 1e-3);
}

TEST(IntentScaling, ReachesFullScaleWhenThePushIsKept)
{
  IntentScaling shaper{IntentScalingConfig{}};
  double previous = 0.0;
  const RobotContext context;
  for (int tick = 0; tick < 250; ++tick)
  {
    const double current = shaper.update(push(1.0), context, kDt).linear.x();
    EXPECT_GE(current, previous - 1e-12);
    previous = current;
  }
  EXPECT_DOUBLE_EQ(previous, 1.0);
  EXPECT_NEAR(shaper.consistency(), 1.0, 1e-9);
}

TEST(IntentScaling, TakesAboutOneAndAHalfSecondsToReachFullScale)
{
  IntentScaling shaper{IntentScalingConfig{}};
  EXPECT_LT(hold(shaper, push(1.0), 1.0).linear.x(), 1.0);
  EXPECT_DOUBLE_EQ(hold(shaper, push(1.0), 1.0).linear.x(), 1.0);
}

TEST(IntentScaling, NeverLeavesUnitScale)
{
  IntentScaling shaper{IntentScalingConfig{}};
  const auto output = hold(shaper, push(3.0, 4.0), 3.0);
  EXPECT_NEAR(output.linear.norm(), 1.0, 1e-9);
  EXPECT_NEAR(output.linear.x() / output.linear.y(), 0.75, 1e-9);
}

TEST(IntentScaling, StartsSlowAgainAfterARelease)
{
  IntentScaling shaper{IntentScalingConfig{}};
  hold(shaper, push(1.0), 3.0);
  EXPECT_DOUBLE_EQ(shaper.scale(), 1.0);

  EXPECT_EQ(shaper.update(push(0.0), RobotContext{}, kDt).linear.norm(), 0.0);
  EXPECT_NEAR(shaper.update(push(1.0), RobotContext{}, kDt).linear.x(), 0.4, 1e-3);
}

TEST(IntentScaling, DoesNotBoostAStickMovedBackAndForth)
{
  IntentScaling shaper{IntentScalingConfig{}};
  const RobotContext context;
  for (int tick = 0; tick < 500; ++tick)
  {
    const double x = (tick / 10) % 2 == 0 ? 1.0 : -1.0; // reverses every 0.1 s
    shaper.update(push(x), context, kDt);
  }
  EXPECT_DOUBLE_EQ(shaper.scale(), 0.4);
}

TEST(IntentScaling, DoesNotBoostAGentlePush)
{
  IntentScaling shaper{IntentScalingConfig{}};
  hold(shaper, push(0.3), 3.0);
  EXPECT_DOUBLE_EQ(shaper.scale(), 0.4);
}

TEST(IntentScaling, StartsSlowAgainWhenThePushTurnsBack)
{
  IntentScaling shaper{IntentScalingConfig{}};
  hold(shaper, push(1.0), 3.0);
  EXPECT_NEAR(shaper.update(push(-1.0), RobotContext{}, kDt).linear.x(), -0.4, 1e-3);
}

TEST(IntentScaling, EasesOffWhenThePushTurnsSideways)
{
  IntentScaling shaper{IntentScalingConfig{}};
  hold(shaper, push(1.0), 3.0);
  const double turning = shaper.update(push(0.0, 1.0), RobotContext{}, kDt).linear.y();
  EXPECT_GT(turning, 0.9);
  EXPECT_LT(hold(shaper, push(0.0, 1.0), 0.2).linear.y(), turning);
  EXPECT_DOUBLE_EQ(hold(shaper, push(0.0, 1.0), 2.0).linear.y(), 1.0);
}

TEST(IntentScaling, LeavesAngularUntouched)
{
  IntentScaling shaper{IntentScalingConfig{}};
  auto input = push(1.0);
  input.angular = Eigen::Vector3d(0.0, 0.0, 0.7);
  EXPECT_EQ(shaper.update(input, RobotContext{}, kDt).angular, input.angular);
}

TEST(IntentScaling, FollowsItsTuning)
{
  IntentScalingConfig config;
  config.min_scale = 0.2;
  config.gain = 4.0;
  IntentScaling shaper{config};
  EXPECT_NEAR(shaper.update(push(1.0), RobotContext{}, kDt).linear.x(), 0.2, 1e-2);
  EXPECT_DOUBLE_EQ(hold(shaper, push(1.0), 0.8).linear.x(), 1.0);
}

class IntentScalingManager : public ::testing::Test
{
protected:
  void SetUp() override
  {
    manager_core::ManagerConfig config;
    config.inputs = {{manager_core::InputSource::JOYSTICK, 0.2, true}};
    config.rate_limiter.max_linear_acceleration = 0.0;
    config.rate_limiter.max_angular_acceleration = 0.0;
    manager.configure(config);
  }

  // Streams a joystick command for `seconds`, as the node does, and returns the last output.
  double drive(double x, double seconds)
  {
    std::optional<CartesianCommand> output;
    for (int tick = 0; tick < static_cast<int>(std::lround(seconds / kDt)); ++tick)
    {
      now += kDt;
      manager.setInputCommand(manager_core::InputSource::JOYSTICK, push(x), now);
      output = manager.update(now, kDt, RobotContext{});
    }
    return output ? output->linear.x() : std::nan("");
  }

  manager_core::Manager manager;
  double now{100.0};
};

TEST_F(IntentScalingManager, IsSelectedByItsModeRequest)
{
  EXPECT_FALSE(manager.setMode("behaviour/intent_scaling/extra"));
  ASSERT_TRUE(manager.setMode("behaviour/intent_scaling"));
  EXPECT_NEAR(drive(1.0, kDt), 0.4, 1e-3);
  EXPECT_DOUBLE_EQ(drive(1.0, 3.0), 1.0);
}

TEST_F(IntentScalingManager, PassthroughKeepsTheCommandAsSent)
{
  ASSERT_TRUE(manager.setMode("behaviour/passthrough"));
  EXPECT_DOUBLE_EQ(drive(1.0, kDt), 1.0);
}

TEST_F(IntentScalingManager, StartsSlowAgainAfterTheInputTimesOut)
{
  ASSERT_TRUE(manager.setMode("behaviour/intent_scaling"));
  drive(1.0, 3.0);
  now += 1.0;
  EXPECT_FALSE(manager.update(now, kDt, RobotContext{}).has_value());
  EXPECT_NEAR(drive(1.0, kDt), 0.4, 1e-3);
}

TEST_F(IntentScalingManager, StartsSlowWhenSelectedAgain)
{
  ASSERT_TRUE(manager.setMode("behaviour/intent_scaling"));
  drive(1.0, 3.0);
  ASSERT_TRUE(manager.setMode("behaviour/intent_scaling"));
  EXPECT_NEAR(drive(1.0, kDt), 0.4, 1e-3);
}

TEST_F(IntentScalingManager, ReportsItsScaleOnlyWhileSelected)
{
  EXPECT_FALSE(manager.intentScale().has_value());
  ASSERT_TRUE(manager.setMode("behaviour/intent_scaling"));
  EXPECT_NEAR(*manager.intentScale(), 0.4, 1e-9);
  drive(1.0, 3.0);
  EXPECT_DOUBLE_EQ(*manager.intentScale(), 1.0);
  ASSERT_TRUE(manager.setMode("behaviour/passthrough"));
  EXPECT_FALSE(manager.intentScale().has_value());
}

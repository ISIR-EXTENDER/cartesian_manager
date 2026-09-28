#include <gtest/gtest.h>

#include <cmath>

#include "cartesian_manager/core/manager.hpp"
#include "cartesian_manager/core/shapers/behaviour/shared_control.hpp"

using manager_core::CartesianCommand;
using manager_core::RobotContext;
using manager_core::SharedControl;
using manager_core::SharedControlConfig;
using manager_core::SharedControlGoal;

namespace
{
  constexpr double kDt = 0.01;

  RobotContext robotAt(const Eigen::Vector3d &position,
                       const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity())
  {
    RobotContext context;
    context.ee_pose.position = position;
    context.ee_pose.orientation = orientation;
    context.ee_pose.frame_id = "base_link";
    context.command_scale.linear = 0.1;
    context.command_scale.angular = 0.2;
    return context;
  }

  CartesianCommand twist(const Eigen::Vector3d &linear,
                         const Eigen::Vector3d &angular = Eigen::Vector3d::Zero())
  {
    CartesianCommand command;
    command.linear = linear;
    command.angular = angular;
    command.frame_id = "base_link";
    return command;
  }

  SharedControlGoal goal(const std::string &id, const Eigen::Vector3d &position,
                         const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity())
  {
    SharedControlGoal goal;
    goal.id = id;
    goal.position = position;
    goal.orientation = orientation;
    return goal;
  }

  double confidenceOf(const SharedControl &shaper, const std::string &id)
  {
    for (const auto &goal : shaper.goals())
      if (goal.id == id)
        return goal.confidence;
    return -1.0;
  }

  void hold(SharedControl &shaper, const CartesianCommand &input, const RobotContext &context,
            double seconds)
  {
    for (int tick = 0; tick < static_cast<int>(std::lround(seconds / kDt)); ++tick)
      shaper.update(input, context, kDt);
  }

  const Eigen::Vector3d kStart(0.5, 0.0, 0.3);
} // namespace

TEST(SharedControl, PassesTheCommandThroughWithoutGoals)
{
  SharedControl shaper{SharedControlConfig{}};
  const auto linear = shaper.update(twist({0.5, 0.2, 0.0}), robotAt(kStart), kDt);
  EXPECT_TRUE(linear.linear.isApprox(Eigen::Vector3d(0.5, 0.2, 0.0)));
  EXPECT_TRUE(linear.angular.isZero());

  const auto angular = shaper.update(twist({0, 0, 0}, {0.0, 0.0, 0.7}), robotAt(kStart), kDt);
  EXPECT_TRUE(angular.linear.isZero(1e-12));
  EXPECT_TRUE(angular.angular.isApprox(Eigen::Vector3d(0.0, 0.0, 0.7)));
}

TEST(SharedControl, PassesTheCommandThroughBeforeThePoseIsKnown)
{
  SharedControlConfig config;
  config.goals = {goal("cup", kStart + Eigen::Vector3d(0.2, 0.0, 0.0))};
  SharedControl shaper{config};
  RobotContext context = robotAt(kStart);
  context.ee_pose.frame_id.clear();
  hold(shaper, twist({1.0, 0.0, 0.0}), context, 1.0);
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "cup"), 0.0);
}

TEST(SharedControl, RaisesTheConfidenceOfTheGoalThePushPointsAt)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0)),
                      goal("behind", kStart - Eigen::Vector3d(0.3, 0.0, 0.0))});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 0.5);
  EXPECT_GT(confidenceOf(shaper, "ahead"), 0.5);
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "behind"), 0.0);
}

TEST(SharedControl, IgnoresASlowPush)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  // 0.1 of 0.1 m/s is 0.01 m/s, under a fifth of v_j_max.
  hold(shaper, twist({0.1, 0.0, 0.0}), robotAt(kStart), 1.0);
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "ahead"), 0.0);
}

TEST(SharedControl, FreezesConfidencesNearAGoal)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("near", kStart + Eigen::Vector3d(0.01, 0.0, 0.0)),
                      goal("far", kStart + Eigen::Vector3d(0.0, 0.3, 0.0))});
  hold(shaper, twist({0.0, 0.6, 0.0}), robotAt(kStart), 0.5);
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "far"), 0.0);
}

TEST(SharedControl, AmplifiesOnlyTheGoalAlignedComponentFarFromTheGoal)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);
  ASSERT_DOUBLE_EQ(confidenceOf(shaper, "ahead"), 1.0);

  const auto output = shaper.update(twist({0.3, 0.2, 0.0}), robotAt(kStart), kDt);
  EXPECT_NEAR(output.linear.x(), 0.6, 1e-9);
  EXPECT_NEAR(output.linear.y(), 0.2, 1e-9);
}

TEST(SharedControl, DoesNotAmplifyInsideTheInnerRadius)
{
  SharedControl shaper{SharedControlConfig{}};
  const Eigen::Vector3d target = kStart + Eigen::Vector3d(0.3, 0.0, 0.0);
  shaper.setGoals({goal("ahead", target)});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);

  const auto output =
      shaper.update(twist({0.3, 0.0, 0.0}), robotAt(target - Eigen::Vector3d(0.01, 0, 0)), kDt);
  EXPECT_NEAR(output.linear.x(), 0.3, 1e-9);
}

TEST(SharedControl, TurnsTheWristToReachTheGoalOrientationWithThePosition)
{
  const Eigen::Quaterniond turned(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0), turned)});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);

  // Full assistance at 0.22 m: 0.5 m/s-scale push, doubled, is 0.1 m/s; 0.1 * 0.5 rad / 0.2 m = 0.25 rad/s.
  const RobotContext context = robotAt(kStart + Eigen::Vector3d(0.08, 0.0, 0.0));
  const auto output = shaper.update(twist({0.5, 0.0, 0.0}), context, kDt);
  EXPECT_NEAR(output.linear.x(), 1.0, 1e-9);
  EXPECT_NEAR(output.angular.z(), 0.25 / 0.2, 1e-6);
  EXPECT_NEAR(output.angular.x(), 0.0, 1e-9);
}

TEST(SharedControl, BlendsTheIncomingAngularCommandAwayWithDistance)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);

  // Halfway through the 0.02-0.04 m ramp, half the jaco baseline survives; the goal needs no turn.
  const RobotContext context = robotAt(kStart + Eigen::Vector3d(0.27, 0.0, 0.0));
  const auto output = shaper.update(twist({0.2, 0.0, 0.0}, {0.0, 0.0, 0.4}), context, kDt);
  EXPECT_NEAR(output.angular.z(), 0.2, 1e-9);
  EXPECT_NEAR(output.linear.x(), 0.3, 1e-9);
}

TEST(SharedControl, PivotsAboutTheGoalInRotationMode)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);

  // 0.5 of 0.2 rad/s about z, 0.3 m from the goal along x: v = (0.3, 0, 0) x (0, 0, 0.1) = -0.03 y m/s.
  const auto output = shaper.update(twist({0, 0, 0}, {0.0, 0.0, 0.5}), robotAt(kStart), kDt);
  EXPECT_NEAR(output.angular.z(), 0.5, 1e-9);
  EXPECT_NEAR(output.linear.y(), -0.03 / 0.1, 1e-9);
  EXPECT_NEAR(confidenceOf(shaper, "ahead"), 1.0, 1e-12);
}

TEST(SharedControl, AmplifiesRotationTowardsTheGoalOrientation)
{
  const Eigen::Quaterniond turned(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0), turned)});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);

  const auto output = shaper.update(twist({0, 0, 0}, {0.0, 0.0, 0.3}), robotAt(kStart), kDt);
  EXPECT_NEAR(output.angular.z(), 0.3 * (1.0 + 2.0), 1e-9);
}

TEST(SharedControl, FollowsTheLiveCommandScale)
{
  const Eigen::Quaterniond turned(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("ahead", kStart + Eigen::Vector3d(0.3, 0.0, 0.0), turned)});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);

  RobotContext context = robotAt(kStart + Eigen::Vector3d(0.08, 0.0, 0.0));
  context.command_scale.linear = 0.2;
  const auto output = shaper.update(twist({0.5, 0.0, 0.0}), context, kDt);
  EXPECT_NEAR(output.angular.z(), (0.2 * 0.5 / 0.2) / 0.2, 1e-6);
}

TEST(SharedControl, KeepsTheConfidenceOfAGoalRedetectedNearby)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("goal_0", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 0.3);
  const double before = confidenceOf(shaper, "goal_0");
  ASSERT_GT(before, 0.0);

  // Now second in the set and 2 cm away: the match follows the position, not the index.
  shaper.setGoals({goal("goal_0", kStart + Eigen::Vector3d(0.0, 0.3, 0.0)),
                   goal("goal_1", kStart + Eigen::Vector3d(0.32, 0.0, 0.0))});
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "goal_1"), before);
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "goal_0"), 0.0);
}

TEST(SharedControl, StartsAGoalThatJumpedFarFromZero)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("goal_0", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 0.3);
  ASSERT_GT(confidenceOf(shaper, "goal_0"), 0.0);

  shaper.setGoals({goal("goal_0", kStart + Eigen::Vector3d(0.3, 0.1, 0.0))});
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "goal_0"), 0.0);
}

TEST(SharedControl, DropsGoalsLeftOutOfTheNextSet)
{
  SharedControl shaper{SharedControlConfig{}};
  shaper.setGoals({goal("goal_0", kStart + Eigen::Vector3d(0.3, 0.0, 0.0)),
                   goal("goal_1", kStart + Eigen::Vector3d(0.0, 0.3, 0.0))});
  shaper.setGoals({goal("goal_0", kStart + Eigen::Vector3d(0.0, 0.3, 0.0))});
  EXPECT_EQ(shaper.goals().size(), 1u);
  shaper.setGoals({});
  EXPECT_TRUE(shaper.goals().empty());
}

TEST(SharedControl, KeepsConfidencesWhenRetuned)
{
  SharedControlConfig config;
  config.goals = {goal("bin", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))};
  SharedControl shaper{config};
  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 0.3);
  const double before = confidenceOf(shaper, "bin");
  ASSERT_GT(before, 0.0);

  config.gamma = 3.0;
  shaper.configure(config);
  EXPECT_DOUBLE_EQ(confidenceOf(shaper, "bin"), before);
}

TEST(SharedControl, SoftGoalMovesTowardsTheLikelyGoal)
{
  SharedControl shaper{SharedControlConfig{}};
  const Eigen::Vector3d target = kStart + Eigen::Vector3d(0.3, 0.0, 0.0);
  shaper.setGoals({goal("ahead", target)});

  auto state = shaper.state(robotAt(kStart));
  ASSERT_EQ(state.goal_ids.size(), 2u);
  EXPECT_EQ(state.goal_ids.front(), "agnostic");
  EXPECT_DOUBLE_EQ(state.confidences.front(), 1.0);
  EXPECT_TRUE(state.soft_goal.position.isApprox(kStart));

  hold(shaper, twist({0.6, 0.0, 0.0}), robotAt(kStart), 2.0);
  state = shaper.state(robotAt(kStart));
  EXPECT_DOUBLE_EQ(state.confidences.front(), 0.0);
  EXPECT_TRUE(state.soft_goal.position.isApprox(target));
}

TEST(SharedControlManager, KeepsConfidencesAcrossModeChangesUntilReset)
{
  manager_core::Manager manager;
  manager_core::ManagerConfig config;
  config.inputs = {{manager_core::InputSource::TABLET, 0.2, true}};
  manager.configure(config);
  manager.setSharedControlGoals({goal("goal_1", kStart + Eigen::Vector3d(0.3, 0.0, 0.0))});
  const RobotContext context = robotAt(kStart);

  ASSERT_TRUE(manager.setMode("behaviour/shared_control"));
  double now = 0.0;
  for (int tick = 0; tick < 30; ++tick, now += kDt)
  {
    manager.setInputCommand(manager_core::InputSource::TABLET, twist({0.6, 0, 0}), now);
    manager.update(now, kDt, context);
  }
  const double before = manager.sharedControlState(context)->confidences.at(1);
  ASSERT_GT(before, 0.0);

  ASSERT_TRUE(manager.setMode("geometric/jaco"));
  ASSERT_TRUE(manager.setMode("behaviour/passthrough"));
  EXPECT_FALSE(manager.sharedControlState(context));
  ASSERT_TRUE(manager.setMode("behaviour/shared_control"));
  EXPECT_DOUBLE_EQ(manager.sharedControlState(context)->confidences.at(1), before);

  ASSERT_TRUE(manager.setMode("behaviour/shared_control/reset"));
  EXPECT_EQ(manager.sharedControlState(context)->confidences.size(), 1u);
  EXPECT_FALSE(manager.setMode("behaviour/shared_control/other"));
}

TEST(SharedControlManager, UsesTheSnakeAngularCommandAsTheTranslationBaseline)
{
  manager_core::Manager manager;
  manager_core::ManagerConfig config;
  config.inputs = {{manager_core::InputSource::TABLET, 0.2, true}};
  manager.configure(config);
  const Eigen::Vector3d target = kStart + Eigen::Vector3d(0.3, 0.0, 0.0);
  manager.setSharedControlGoals({goal("goal_0", target)});
  ASSERT_TRUE(manager.setMode("geometric/snake"));
  ASSERT_TRUE(manager.setMode("behaviour/shared_control"));

  double now = 0.0;
  const auto push = [&](const RobotContext &context, double dt) {
    now += dt;
    manager.setInputCommand(manager_core::InputSource::TABLET, twist({0.6, 0.0, 0.0}), now);
    return *manager.update(now, dt, context);
  };

  // Far and sure of the goal, whose orientation is already reached: assistance replaces snake's turn.
  CartesianCommand far;
  for (int tick = 0; tick < 200; ++tick)
    far = push(robotAt(kStart), kDt);
  EXPECT_NEAR(far.angular.norm(), 0.0, 1e-9);

  // Inside the inner radius: snake's gain * z x v, (0, 1.8, 0), normalised to unit scale.
  const auto near = push(robotAt(target - Eigen::Vector3d(0.01, 0.0, 0.0)), 1.0);
  EXPECT_TRUE(near.angular.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-9));
}

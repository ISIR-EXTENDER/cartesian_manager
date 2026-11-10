#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "cartesian_manager/core/input_manager.hpp"
#include "cartesian_manager/ros/parameter_parsing.hpp"

using manager_core::InputSource;
using ros_cartesian_manager::parseManagerConfig;
using ros_cartesian_manager::updatedParamsForRequest;

TEST(HandJoystick, ParsesIndependentChannel)
{
  cartesian_manager::Params params;
  params.inputs.sources = {"joystick", "hand_joystick"};
  params.inputs.hand_joystick.timeout_sec = 0.35;
  const auto config = parseManagerConfig(params);
  ASSERT_EQ(config.manager.inputs.size(), 2u);
  EXPECT_EQ(config.manager.inputs[0].source, InputSource::JOYSTICK);
  EXPECT_EQ(config.manager.inputs[1].source, InputSource::HAND_JOYSTICK);
  EXPECT_DOUBLE_EQ(config.manager.inputs[1].timeout_sec, 0.35);
  EXPECT_TRUE(config.manager.inputs[1].enabled);
  EXPECT_EQ(config.topics.hand_joystick_command, "/hand_joystick_cartesian_command");
  EXPECT_EQ(config.topics.joystick_command, "/joystick_cartesian_command");
}

TEST(HandJoystick, ValidatesStartupAndRuntimeParameters)
{
  cartesian_manager::Params params;
  params.inputs.sources = {"hand_joystick"};
  for (double invalid : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN()})
  {
    const auto updated = updatedParamsForRequest(
        params, {rclcpp::Parameter("inputs.hand_joystick.timeout_sec", invalid)});
    EXPECT_THROW(parseManagerConfig(updated), std::invalid_argument);
  }
  const auto updated = updatedParamsForRequest(
      params, {rclcpp::Parameter("inputs.hand_joystick.timeout_sec", 0.4),
               rclcpp::Parameter("inputs.hand_joystick.enabled", false)});
  const auto config = parseManagerConfig(updated);
  EXPECT_DOUBLE_EQ(config.manager.inputs[0].timeout_sec, 0.4);
  EXPECT_FALSE(config.manager.inputs[0].enabled);
  params.topics.hand_joystick_command.clear();
  EXPECT_THROW(parseManagerConfig(params), std::invalid_argument);
}

TEST(HandJoystick, ExpiresAndDisablesIndependentlyOfPhysicalJoystick)
{
  manager_core::InputManager inputs;
  inputs.configureInputChannels({{InputSource::JOYSTICK, 1.0, true},
                                 {InputSource::HAND_JOYSTICK, 0.2, true}});
  manager_core::CartesianVelocity joystick;
  joystick.linear.x() = 0.1;
  manager_core::CartesianVelocity hand;
  hand.linear.y() = 0.2;
  ASSERT_TRUE(inputs.setCommand(InputSource::JOYSTICK, joystick, 1.0));
  ASSERT_TRUE(inputs.setCommand(InputSource::HAND_JOYSTICK, hand, 1.0));
  const manager_core::RobotContext context;
  auto combined = inputs.getFullCommand(1.1, context);
  ASSERT_TRUE(combined);
  EXPECT_DOUBLE_EQ(combined->linear.x(), 0.1);
  EXPECT_DOUBLE_EQ(combined->linear.y(), 0.2);
  auto expired = inputs.getFullCommand(1.3, context);
  ASSERT_TRUE(expired);
  EXPECT_DOUBLE_EQ(expired->linear.x(), 0.1);
  EXPECT_DOUBLE_EQ(expired->linear.y(), 0.0);
  ASSERT_TRUE(inputs.setCommand(InputSource::HAND_JOYSTICK, hand, 1.3));
  inputs.disableInputChannel(InputSource::HAND_JOYSTICK);
  EXPECT_FALSE(inputs.hasValidCommand(InputSource::HAND_JOYSTICK, 1.4));
  EXPECT_TRUE(inputs.hasValidCommand(InputSource::JOYSTICK, 1.4));
  inputs.enableInputChannel(InputSource::HAND_JOYSTICK);
  EXPECT_TRUE(inputs.hasValidCommand(InputSource::HAND_JOYSTICK, 1.4));
}

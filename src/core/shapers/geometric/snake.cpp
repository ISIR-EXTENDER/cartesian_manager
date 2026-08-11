#include "cartesian_manager/core/shapers/geometric/snake.hpp"

namespace manager_core
{
  SnakeShaper::SnakeShaper(const SnakeShaperConfig &config) : config_(config)
  {
  }

  CartesianVelocity SnakeShaper::update(const CartesianVelocity &input, const RobotContext &context,
                                       double /*dt_sec*/)
  {
    CartesianVelocity output = input;

    const Eigen::Vector3d tool_z_axis = context.ee_pose.orientation.toRotationMatrix().col(2);
    output.angular = config_.gain * tool_z_axis.cross(input.linear) + input.angular;

    return output;
  }

  void SnakeShaper::reset()
  {
  }
} // namespace manager_core

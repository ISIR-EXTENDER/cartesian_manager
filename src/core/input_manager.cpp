#include "cartesian_manager/core/input_manager.hpp"

namespace manager_core
{
  void InputManager::setFramesConfig(const std::string &ee_frame, const std::string &base_frame,
                                     const std::string &hybrid_frame)
  {
    frames_names.base_frame = base_frame;
    frames_names.ee_frame = ee_frame;
    frames_names.hybrid_frame = hybrid_frame;
  }

  void InputManager::setFramesConfig(const FramesConfig &frame_names)
  {
    frames_names = frame_names;
  }

  void InputManager::addInputChannel(InputSource source, double timeout_sec, bool enabled)
  {
    auto &channel = inputs_[source];
    channel.timeout = timeout_sec;
    channel.enabled = enabled;
  }

  void InputManager::enableInputChannel(InputSource source)
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return;
    }

    input->second.enabled = true;
  }

  void InputManager::disableInputChannel(InputSource source)
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return;
    }

    input->second.enabled = false;
  }

  bool InputManager::isInputChannelEnabled(InputSource source) const
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return false;
    }

    return input->second.enabled;
  }

  bool InputManager::hasInputChannel(InputSource source) const
  {
    return inputs_.find(source) != inputs_.end();
  }

  bool InputManager::setCommand(InputSource source, const CartesianVelocity &command,
                                double stamp_sec)
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return false;
    }

    input->second.latest.command = command;
    input->second.latest.stamp_sec = stamp_sec;
    input->second.latest.received = true;
    return true;
  }

  bool InputManager::hasValidCommand(InputSource source, double now_sec) const
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return false;
    }

    const auto &channel = input->second;
    if (!channel.enabled || !channel.latest.received)
    {
      return false;
    }

    return now_sec - channel.latest.stamp_sec <= channel.timeout;
  }

  std::optional<CartesianVelocity> InputManager::getCommand(InputSource source,
                                                            double now_sec) const
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end() || !hasValidCommand(source, now_sec))
    {
      return std::nullopt;
    }

    return input->second.latest.command;
  }

  std::vector<InputSource> InputManager::getValidSources(double now_sec) const
  {
    std::vector<InputSource> sources;
    sources.reserve(inputs_.size());

    for (const auto &[source, _] : inputs_)
    {
      if (hasValidCommand(source, now_sec))
      {
        sources.push_back(source);
      }
    }

    return sources;
  }

  void InputManager::clearCommand(InputSource source)
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return;
    }

    input->second.latest = TimedCartesianCommand{};
  }

  void InputManager::clearAllCommands()
  {
    for (auto &[_, channel] : inputs_)
    {
      channel.latest = TimedCartesianCommand{};
    }
  }

  void InputManager::clearInputChannels()
  {
    inputs_.clear();
  }

  std::optional<CartesianVelocity> InputManager::commandInBaseFrame(
      const CartesianVelocity &input, const RobotContext &context) const
  {
    CartesianVelocity command = input;
    if (command.frame_id.empty() || command.frame_id == frames_names.base_frame)
    {
      command.frame_id = frames_names.base_frame;
      return command;
    }

    const CartesianPose *pose = nullptr;
    if (command.frame_id == frames_names.ee_frame)
    {
      pose = &context.ee_pose;
    }
    else if (command.frame_id == frames_names.hybrid_frame)
    {
      pose = &context.hybrid_pose;
    }
    else
    {
      return std::nullopt;
    }

    auto rotation = pose->orientation;
    if (rotation.norm() == 0.0)
    {
      return std::nullopt;
    }
    rotation.normalize();

    command.angular = rotation * input.angular;
    command.frame_id = frames_names.base_frame;
    return command;
  }

  std::optional<CartesianVelocity> InputManager::getFullCommand(double now_sec,
                                                                const RobotContext &context) const
  {
    CartesianVelocity command;
    command.frame_id = frames_names.base_frame;
    double total_weight = 0.0;

    for (const auto &[source, channel] : inputs_)
    {
      if (!hasValidCommand(source, now_sec) || channel.weight <= 0.0)
      {
        continue;
      }

      const auto transformed_command = commandInBaseFrame(channel.latest.command, context);
      if (!transformed_command)
      {
        continue;
      }

      command.linear += channel.weight * transformed_command->linear;
      command.angular += channel.weight * transformed_command->angular;
      total_weight += channel.weight;
    }

    if (total_weight <= 0.0)
    {
      return std::nullopt;
    }

    command.linear /= total_weight;
    command.angular /= total_weight;

    return command;
  }
} // namespace manager_core

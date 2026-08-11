#include "cartesian_manager/core/input_manager.hpp"

namespace manager_core
{
  void InputManager::setFrameId(const std::string frame_id)
  {
    input_frame_id = frame_id;
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

  bool InputManager::checkFrameId(const CartesianVelocity &command) const
  {
    return command.frame_id == input_frame_id;
  }

  bool InputManager::setCommand(InputSource source, const CartesianVelocity &command,
                                double stamp_sec)
  {
    auto input = inputs_.find(source);
    if (input == inputs_.end())
    {
      return false;
    }
    if (!checkFrameId(command))
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

  std::optional<CartesianVelocity> InputManager::getFullCommand(double now_sec) const
  {
    CartesianVelocity command;
    double total_weight = 0.0;

    for (const auto &[source, channel] : inputs_)
    {
      if (!hasValidCommand(source, now_sec) || channel.weight <= 0.0)
      {
        continue;
      }

      if (command.frame_id.empty())
      {
        command.frame_id = channel.latest.command.frame_id;
      }
      else if (command.frame_id != channel.latest.command.frame_id)
      {
        continue;
      }

      command.linear += channel.weight * channel.latest.command.linear;
      command.angular += channel.weight * channel.latest.command.angular;
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

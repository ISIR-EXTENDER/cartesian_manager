#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  class InputManager
  {

  public:
    void setFrameId(const std::string frame_id);
    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);

    void enableInputChannel(InputSource source);
    void disableInputChannel(InputSource source);

    bool isInputChannelEnabled(InputSource source) const;
    bool hasInputChannel(InputSource source) const;

    bool checkFrameId(const CartesianVelocity &command) const;

    bool setCommand(InputSource source, const CartesianVelocity &command, double stamp_sec);
    bool hasValidCommand(InputSource source, double now_sec) const;
    std::optional<CartesianVelocity> getCommand(InputSource source, double now_sec) const;

    std::vector<InputSource> getValidSources(double now_sec) const;

    void clearCommand(InputSource source);
    void clearAllCommands();
    void clearInputChannels();

    std::optional<CartesianVelocity> getFullCommand(double now_sec) const;

  private:
    std::unordered_map<InputSource, InputChannel> inputs_;
    std::string input_frame_id;
  };
} // namespace manager_core

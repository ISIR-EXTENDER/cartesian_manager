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
    void setFramesConfig(const std::string &ee_frame, const std::string &base_frame,
                         const std::string &hybrid_frame);
    void setFramesConfig(const FramesConfig &frame_names);
    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);

    void enableInputChannel(InputSource source);
    void disableInputChannel(InputSource source);

    bool isInputChannelEnabled(InputSource source) const;
    bool hasInputChannel(InputSource source) const;

    bool setCommand(InputSource source, const CartesianVelocity &command, double stamp_sec);
    bool hasValidCommand(InputSource source, double now_sec) const;
    std::optional<CartesianVelocity> getCommand(InputSource source, double now_sec) const;

    std::vector<InputSource> getValidSources(double now_sec) const;

    void clearCommand(InputSource source);
    void clearAllCommands();
    void clearInputChannels();

    std::optional<CartesianVelocity> getFullCommand(double now_sec,
                                                    const RobotContext &context) const;

  private:
    std::optional<CartesianVelocity> commandInBaseFrame(const CartesianVelocity &command,
                                                        const RobotContext &context) const;

    std::unordered_map<InputSource, InputChannel> inputs_;
    FramesConfig frames_names;
  };
} // namespace manager_core

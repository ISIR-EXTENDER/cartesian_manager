#pragma once

#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <variant>

#include "cartesian_manager/core/input_manager.hpp"

#include "cartesian_manager/core/ratelimiter.hpp"
#include "cartesian_manager/core/shapers/behaviour/joint_target.hpp"
#include "cartesian_manager/core/shapers/behaviour/pose_target.hpp"
#include "cartesian_manager/core/shapers/geometric/jaco.hpp"
#include "cartesian_manager/core/shapers/geometric/snake.hpp"
#include "cartesian_manager/core/shapers/shaper.hpp"
#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  struct ManagerConfig
  {
    JacoShaperConfig jaco;
    SnakeShaperConfig snake;
    FramesConfig frames;
    JointTargetConfig joint_targets;
    PoseTargetConfig pose_targets;
    RateLimiterConfig rate_limiter;
    std::vector<InputConfig> inputs;
  };

  class Manager
  {
  public:
    void configure(const ManagerConfig &config);
    void updateTuning(const ManagerConfig &config);
    void addInputChannel(InputSource source, double timeout_sec, bool enabled = true);
    void configureInputChannels(const std::vector<InputConfig> &channels);
    void clearInputChannels();
    void updateJointTargetName(std::string& joint_target_name);

    void enableInput(InputSource source);
    void disableInput(InputSource source);

    bool setInputCommand(InputSource source, const CartesianVelocity &command, double stamp_sec);
    std::vector<InputSource> getValidInputSources(double now_sec) const;

    bool setMode(const std::string &mode_request);
    bool setPoseTarget(const CartesianPose &target, std::string *error = nullptr);
    bool setJointTarget(const std::string &target_name, std::string *error = nullptr);
    // std::optional<JointTargetCommand> activeJointTargetCommand(const RobotContext &context)
    // const;
    JointVelocityCommand activeJointTargetCommand(const RobotContext &robot_context);

    Geometrics geometricState() const;
    Behaviours behaviourState() const;
    std::string activeTargetName() const;

    std::variant<CartesianCommand, JointVelocityCommand> update(double now_sec, double dt_sec,
                                                              const RobotContext &context);

    InputManager getInputManager() const
    {
      return input_manager_;
    };

  private:
    void applyGeometric(CartesianVelocity &command, const RobotContext &context, double dt_sec);
    void applyBehaviour(CartesianVelocity &command, const RobotContext &context, double dt_sec);

    void registerGeometricShaper(Geometrics state, std::unique_ptr<Shaper> shaper);
    void registerBehaviour(Behaviours state, std::unique_ptr<Shaper> shaper);
    const JointTargetCommand *jointTargetByName(const std::string &target_name) const;

    Geometrics geometric_state_{Geometrics::BOTH};
    Behaviours behaviour_state_{Behaviours::PASSTHROUGH};
    JointTargetConfig joint_target_config_;
    JointTarget joint_target_;
    RateLimiter rate_limiter_;

    InputManager input_manager_;

    std::unordered_map<Geometrics, std::unique_ptr<Shaper>> geometric_shapers_;
    std::unordered_map<Behaviours, std::unique_ptr<Shaper>> behaviours_;
  };
} // namespace manager_core

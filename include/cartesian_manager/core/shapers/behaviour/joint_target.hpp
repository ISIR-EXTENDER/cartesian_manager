#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "cartesian_manager/core/shapers/shaper.hpp"

namespace manager_core
{

  struct JointTargetCommand
  {
    std::string name;
    std::vector<std::string> joint_names;
    Eigen::VectorXd positions;
  };

  struct JointVelocityCommand
  {
    std::vector<std::string> joint_names;
    Eigen::VectorXd joint_speed;
  };

  struct JointTargetConfig // Bibliothèque de positions articulaires et des paramètres
  {
    std::vector<std::string> joint_names;      // référence des noms des joints q1, q2, ...
    std::vector<std::string> target_names;     // "home", ...
    std::vector<JointTargetCommand> positions; // Positions articulaires associées
    double velocity_gain;
    double max_velocity;
    double position_tolerance;
    double task_weight;
  };

  class JointTarget : public Shaper
  {
  public:
    JointTarget() = default;
    explicit JointTarget(const JointTargetConfig &config);

    void reset() override;
    void configure(const JointTargetConfig &config);
    void addJointTarget(const std::string &name, const Eigen::VectorXd &joints);

    std::string name() const override;
    std::string activeTargetName() const;
    std::optional<std::string> validate(const RobotContext &context) const override;

    bool start(const std::string &target_name, const RobotContext &context,
               std::string *error = nullptr) override;
    bool start(const Eigen::VectorXd &target, std::string *error = nullptr);
    bool isComplete(const RobotContext &context) const override;
    bool hasTarget(const std::string &name) const;
    bool isActive() const;
    bool setActiveTarget(const std::string &name);

    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;
    const JointTargetCommand *activeTarget() const;
    JointVelocityCommand getJointVelocityCommand(const RobotContext &context);

  private:
    std::unordered_map<std::string, JointTargetCommand> joint_targets_;
    JointTargetConfig config_;
    JointTargetCommand active_target_;

    bool dynamic_target_active_{false};
    bool isActive_{false};
  };

} // namespace manager_core

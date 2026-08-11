#pragma once

#include <optional>
#include <string>
#include <vector>

#include "cartesian_manager/core/shapers/shaper.hpp"

namespace manager_core
{
  struct JointTarget
  {
    std::string name;
    std::vector<double> positions;
  };

  struct JointTargetCommand
  {
    std::string name;
    std::vector<std::string> joint_names;
    std::vector<double> positions;
  };

  struct JointTargetBehaviourConfig
  {
    std::vector<std::string> joint_names;
    std::vector<JointTarget> targets;
  };

  class JointTargetBehaviour : public Shaper
  {
  public:
    explicit JointTargetBehaviour(JointTargetBehaviourConfig config = {});

    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;

    std::string name() const override;
    bool start(const std::string &target_name, const RobotContext &context,
               std::string *error = nullptr) override;

    bool acceptsInputCommand() const override;
    bool setTarget(const std::string &target_name);
    const std::string &activeTargetName() const;

    std::optional<std::string> validate(const RobotContext &context) const override;
    bool isComplete(const RobotContext &context) const override;

  private:
    bool hasValidConfig() const;
    const JointTarget *targetByName(const std::string &target_name) const;
    const JointTarget *activeTarget() const;
    std::optional<Eigen::VectorXd> targetPositions(const JointTarget &target) const;
    std::optional<Eigen::VectorXd> orderedCurrentPositions(const RobotContext &context) const;

    JointTargetBehaviourConfig config_;
    std::string active_target_name_{"home"};
  };
} // namespace manager_core
